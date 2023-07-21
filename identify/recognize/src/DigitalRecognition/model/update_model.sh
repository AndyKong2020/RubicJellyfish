#!/usr/bin/sh
gh release download --pattern '*.zip' --clobber
unzip -o ./BP_ov.zip
unzip -o ./mobileNet_ov.zip
rm ./BP_ov.zip ./mobileNet_ov.zip
echo "Done!"
