xxd -r -p RS9113_WC_BL_0_5_hex RS9113_WC_BL_0_5_hex_bin
objcopy -I binary -O binary --reverse-bytes=4 RS9113_WC_BL_0_5_hex_bin outputfile.bin
cat outputfile.bin | od -v -w1 -tx1 | awk '{print $2;}' |  sed -e '/^$/d' > RS9113_WC_BL_0_5_hex_8
sed -e 's/$/,/g' RS9113_WC_BL_0_5_hex_8 > non_rf_values1.txt
sed -e 's/^/0x/g' non_rf_values1.txt > RS9113_WC_BL_0_5_hex_8
rm non_rf*  RS9113_WC_BL_0_5_hex_bin -rf
