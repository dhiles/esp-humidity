# update nvs partition

## wifi_credentials.csv

key,type,encoding,value
config.wifi,namespace,,
ssid,data,string,"myssid"
password,data,string,"mypassword"

## commands to update nvs partition

python $IDF_PATH/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py generate wifi_credentials.csv nvs.bin 0x6000

$IDF_PATH/components/partition_table/parttool.py --port /dev/ttyUSB0 write_partition --partition-name=nvs --input nvs.bin

## erase partition
$IDF_PATH/components/partition_table/parttool.py --port /dev/ttyACM1 erase_partition --partition-name=nvs

this project needs a components repo which can be setup by cding to the components dir and running:
git clone https://github.com/BoschSensortec/BME280_driver.git
git clone https://github.com/dhiles/esp32-shared.git

## building


idf.py -p /dev/ttyACM0 flash


