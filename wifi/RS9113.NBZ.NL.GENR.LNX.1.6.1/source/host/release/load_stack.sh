rmmod  sdhci_pci
rmmod  sdhci
rmmod  mmc_block
rmmod  mmc_core

modprobe sdhci
modprobe sdhci_pci
modprobe mmc_block
modprobe mmc_core

