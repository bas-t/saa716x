// SPDX-License-Identifier: GPL-2.0+

#include "saa716x_mod.h"

#include "saa716x_gpio_reg.h"
#include "saa716x_greg_reg.h"
#include "saa716x_msi_reg.h"

#include "saa716x_adap.h"
#include "saa716x_boot.h"
#include "saa716x_i2c.h"
#include "saa716x_pci.h"
#include "saa716x_budget.h"
#include "saa716x_gpio.h"
#include "saa716x_priv.h"

#include "si2168.h"
#include "si2157.h"

#define DRIVER_NAME	"SAA716x Budget"

static int saa716x_budget_pci_probe(struct pci_dev *pdev,
				    const struct pci_device_id *pci_id)
{
	struct saa716x_dev *saa716x;
	int err = 0;

	saa716x = kzalloc(sizeof(struct saa716x_dev), GFP_KERNEL);
	if (saa716x == NULL) {
		err = -ENOMEM;
		goto fail0;
	}

	saa716x->pdev		= pdev;
	saa716x->module		= THIS_MODULE;
	saa716x->config		= (struct saa716x_config *) pci_id->driver_data;

	err = saa716x_pci_init(saa716x);
	if (err) {
		pci_err(saa716x->pdev, "PCI Initialization failed");
		goto fail1;
	}

	err = saa716x_cgu_init(saa716x);
	if (err) {
		pci_err(saa716x->pdev, "CGU Init failed");
		goto fail1;
	}

	err = saa716x_jetpack_init(saa716x);
	if (err) {
		pci_err(saa716x->pdev, "Jetpack core initialization failed");
		goto fail2;
	}

	err = saa716x_i2c_init(saa716x);
	if (err) {
		pci_err(saa716x->pdev, "I2C Initialization failed");
		goto fail3;
	}

	saa716x_gpio_init(saa716x);

	err = saa716x_dvb_init(saa716x);
	if (err) {
		pci_err(saa716x->pdev, "DVB initialization failed");
		goto fail4;
	}

	return 0;

fail4:
	saa716x_dvb_exit(saa716x);
fail3:
	saa716x_i2c_exit(saa716x);
fail2:
	saa716x_pci_exit(saa716x);
fail1:
	kfree(saa716x);
fail0:
	return err;
}

static void saa716x_budget_pci_remove(struct pci_dev *pdev)
{
	struct saa716x_dev *saa716x = pci_get_drvdata(pdev);

	saa716x_dvb_exit(saa716x);
	saa716x_i2c_exit(saa716x);
	saa716x_pci_exit(saa716x);
	kfree(saa716x);
}

static irqreturn_t saa716x_budget_pci_irq(int irq, void *dev_id)
{
	struct saa716x_dev *saa716x	= (struct saa716x_dev *) dev_id;

	u32 stat_h, stat_l, mask_h, mask_l;

	stat_l = SAA716x_EPRD(MSI, MSI_INT_STATUS_L);
	stat_h = SAA716x_EPRD(MSI, MSI_INT_STATUS_H);
	mask_l = SAA716x_EPRD(MSI, MSI_INT_ENA_L);
	mask_h = SAA716x_EPRD(MSI, MSI_INT_ENA_H);

	pci_dbg(saa716x->pdev, "MSI STAT L=<%02x> H=<%02x>, CTL L=<%02x> H=<%02x>",
		stat_l, stat_h, mask_l, mask_h);

	if (!((stat_l & mask_l) || (stat_h & mask_h)))
		return IRQ_NONE;

	if (stat_l)
		SAA716x_EPWR(MSI, MSI_INT_STATUS_CLR_L, stat_l);
	if (stat_h)
		SAA716x_EPWR(MSI, MSI_INT_STATUS_CLR_H, stat_h);

	if (stat_l & MSI_INT_TAGACK_FGPI_0)
		tasklet_schedule(&saa716x->fgpi[0].tasklet);
	if (stat_l & MSI_INT_TAGACK_FGPI_1)
		tasklet_schedule(&saa716x->fgpi[1].tasklet);
	if (stat_l & MSI_INT_TAGACK_FGPI_2)
		tasklet_schedule(&saa716x->fgpi[2].tasklet);
	if (stat_l & MSI_INT_TAGACK_FGPI_3)
		tasklet_schedule(&saa716x->fgpi[3].tasklet);

	return IRQ_HANDLED;
}

#define SAA716x_MODEL_TBS6281		"TurboSight TBS 6281"
#define SAA716x_DEV_TBS6281		"DVB-T/T2/C"

static int saa716x_tbs6281_frontend_attach(struct saa716x_adapter *adapter,
					   int count)
{
	struct saa716x_dev *dev = adapter->saa716x;
	struct i2c_adapter *i2c_adapter;
	struct si2168_config si2168_config = {};
	struct si2157_config si2157_config = {};

	if (count > 1)
		goto err;

	/* reset */
	saa716x_gpio_set_output(dev, count ? 2 : 16);
	saa716x_gpio_write(dev, count ? 2 : 16, 0);
	msleep(50);
	saa716x_gpio_write(dev, count ? 2 : 16, 1);
	msleep(100);

	/* attach demod */
	si2168_config.i2c_adapter = &i2c_adapter;
	si2168_config.fe = &adapter->fe;
	si2168_config.ts_mode = SI2168_TS_PARALLEL;
	si2168_config.ts_clock_gapped = true;
	adapter->i2c_client_demod =
			dvb_module_probe("si2168", NULL,
					 &dev->i2c[1 - count].i2c_adapter,
					 0x64, &si2168_config);
	if (!adapter->i2c_client_demod)
		goto err;

	/* attach tuner */
	si2157_config.fe = adapter->fe;
	si2157_config.if_port = 1;
	adapter->i2c_client_tuner = dvb_module_probe("si2157", NULL,
						     i2c_adapter,
						     0x60, &si2157_config);
	if (!adapter->i2c_client_tuner) {
		dvb_module_release(adapter->i2c_client_demod);
		goto err;
	}

	pci_dbg(dev->pdev, "%s frontend %d attached",
		dev->config->model_name, count);

	return 0;
err:
	pci_err(dev->pdev, "%s frontend %d attach failed",
		dev->config->model_name, count);
	return -ENODEV;
}

static const struct saa716x_config saa716x_tbs6281_config = {
	.model_name		= SAA716x_MODEL_TBS6281,
	.dev_type		= SAA716x_DEV_TBS6281,
	.adapters		= 2,
	.frontend_attach	= saa716x_tbs6281_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_400,
	.i2c_mode		= SAA716x_I2C_MODE_POLLING,
	.adap_config		= {
		{
			/* adapter 0 */
			.ts_vp   = 6,
			.ts_fgpi = 1
		},
		{
			/* adapter 1 */
			.ts_vp   = 2,
			.ts_fgpi = 3
		},
	},
};

#define SAA716x_MODEL_TBS6285		"TurboSight TBS 6285"
#define SAA716x_DEV_TBS6285		"DVB-T/T2/C"

static int saa716x_tbs6285_frontend_attach(struct saa716x_adapter *adapter,
					   int count)
{
	struct saa716x_dev *dev = adapter->saa716x;
	struct i2c_adapter *i2c_adapter;
	struct si2168_config si2168_config = {};
	struct si2157_config si2157_config = {};

	if (count > 3)
		goto err;

	/* attach demod */
	si2168_config.i2c_adapter = &i2c_adapter;
	si2168_config.fe = &adapter->fe;
	si2168_config.ts_mode = SI2168_TS_SERIAL;
	si2168_config.ts_clock_gapped = true;
	adapter->i2c_client_demod =
			dvb_module_probe("si2168", NULL,
					 ((count == 0) || (count == 1)) ?
					  &dev->i2c[1].i2c_adapter :
					  &dev->i2c[0].i2c_adapter,
					 ((count == 0) || (count == 2)) ?
					  0x64 : 0x66,
					 &si2168_config);
	if (!adapter->i2c_client_demod)
		goto err;

	/* attach tuner */
	si2157_config.fe = adapter->fe;
	si2157_config.if_port = 1;
	adapter->i2c_client_tuner =
			dvb_module_probe("si2157", NULL, i2c_adapter,
					 ((count == 0) || (count == 2)) ?
					  0x62 : 0x60,
					 &si2157_config);
	if (!adapter->i2c_client_tuner) {
		dvb_module_release(adapter->i2c_client_demod);
		goto err;
	}

	pci_dbg(dev->pdev, "%s frontend %d attached",
		dev->config->model_name, count);

	return 0;
err:
	pci_err(dev->pdev, "%s frontend %d attach failed",
		dev->config->model_name, count);
	return -ENODEV;
}

static const struct saa716x_config saa716x_tbs6285_config = {
	.model_name		= SAA716x_MODEL_TBS6285,
	.dev_type		= SAA716x_DEV_TBS6285,
	.adapters		= 4,
	.frontend_attach	= saa716x_tbs6285_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_400,
	.i2c_mode		= SAA716x_I2C_MODE_POLLING,
	.adap_config		= {
		{
			/* adapter 0 */
			.ts_vp   = 2,
			.ts_fgpi = 3
		},
		{
			/* adapter 1 */
			.ts_vp   = 3,
			.ts_fgpi = 2
		},
		{
			/* adapter 2 */
			.ts_vp   = 6,
			.ts_fgpi = 1
		},
		{
			/* adapter 3 */
			.ts_vp   = 5,
			.ts_fgpi = 0
		},
	},
};


static const struct pci_device_id saa716x_budget_pci_table[] = {
	MAKE_ENTRY(TURBOSIGHT_TBS6281, TBS6281,    SAA7160, &saa716x_tbs6281_config),
	MAKE_ENTRY(TURBOSIGHT_TBS6285, TBS6285,    SAA7160, &saa716x_tbs6285_config),
	{ }
};
MODULE_DEVICE_TABLE(pci, saa716x_budget_pci_table);

static struct pci_driver saa716x_budget_pci_driver = {
	.name			= DRIVER_NAME,
	.id_table		= saa716x_budget_pci_table,
	.probe			= saa716x_budget_pci_probe,
	.remove			= saa716x_budget_pci_remove,
};

static int __init saa716x_budget_init(void)
{
	return pci_register_driver(&saa716x_budget_pci_driver);
}

static void __exit saa716x_budget_exit(void)
{
	return pci_unregister_driver(&saa716x_budget_pci_driver);
}

module_init(saa716x_budget_init);
module_exit(saa716x_budget_exit);

MODULE_DESCRIPTION("SAA716x Budget driver");
MODULE_AUTHOR("Manu Abraham");
MODULE_LICENSE("GPL");
