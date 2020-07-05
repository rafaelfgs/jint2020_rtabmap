#! /bin/usr/bash

USER_LOGIN=""
USER_PASS=""
PROBE_WEBSITE="itv.org"

COOKIE_FNAME=/tmp/cookies.txt
TEMP_FILENAME=/tmp/wifi_itv_login_tmp

echo "" > $COOKIE_FNAME; 
echo "" > $TEMP_FILENAME; 

wget -O- --load-cookies $COOKIE_FNAME $PROBE_WEBSITE --append-output=$TEMP_FILENAME >> $TEMP_FILENAME

FORM_LOCATION=$(grep -oP "Location: \K[\w+\:\.\/\?]+" $TEMP_FILENAME)
MAGIC_TOKEN=$(grep -oP "input type=\"hidden\" name=\"magic\" value=\"\K[0-9a-zA-Z]+(?=\">)" $TEMP_FILENAME)

echo -e "Form location: $FORM_LOCATION"
echo -e "Magic token: $MAGIC_TOKEN"


wget -O- --load-cookies $COOKIE_FNAME $PROBE_WEBSITE $FORM_LOCATION --post-data="magic=$MAGIC_TOKEN&username=$USER_LOGIN&password=$USER_PASS" --read-timeout=1
