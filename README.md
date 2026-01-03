# esp-humidity

## deep_sleep and always_on flag setting
mosquittpub -t "acreage/well/cmd/mode" -m "deep_sleep" -r -q 1
mosquittpub -t "acreage/well/cmd/mode" -m "always_on" -r -q 1

## monitor deep_sleep and always_on
minicom -D /dev/ttyACM0 -b 115200

## build
idf.py -p /dev/ttyACM0 flash


