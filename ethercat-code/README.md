#EtherCAT Code

### Igh EtherCAT master Installation

Install the EtherCAT respository based on the instruction given on the link below.
```
https://gitlab.com/etherlab.org/ethercat
```

Add the following Configuration as well
```
./configure --disable-8139too --enable-sii-assign --enable-hrtimer --enable-cycles --disable-eoe
```

### Changes Required
Changes are required in the code for the following case
1. If Motor Manufacturer is different, please update the rated torque, encoder count, gear ratio etc.
1. If the Drive manufacturer is different, please update the Vendor Id ans other things.

### Build the code
To build the code, change your directory to build and run the following command
```
make
```

### Run the code
Under the Build Directory, run the following command.
```
sudo ./ecat_config_publisher
```
