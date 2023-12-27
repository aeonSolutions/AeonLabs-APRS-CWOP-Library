[![Telegram](https://img.shields.io/badge/join-telegram-blue.svg?style=for-the-badge)](https://t.me/+W4rVVa0_VLEzYmI0)
 [![WhatsApp](https://img.shields.io/badge/join-whatsapp-green.svg?style=for-the-badge)](https://chat.whatsapp.com/FkNC7u83kuy2QRA5sqjBVg) 
 [![Donate](https://img.shields.io/badge/donate-$-brown.svg?style=for-the-badge)](http://paypal.me/mtpsilva)
 [![Say Thanks](https://img.shields.io/badge/Say%20Thanks-!-yellow.svg?style=for-the-badge)](https://saythanks.io/to/mtpsilva)
![](https://img.shields.io/github/last-commit/aeonSolutions/aeonlabs-ESP32-C-Base-Firmware-Libraries?style=for-the-badge)
<a href="https://trackgit.com">
<img src="https://us-central1-trackgit-analytics.cloudfunctions.net/token/ping/lgj908xjlweccmzynhbl" alt="trackgit-views" />
</a>
![](https://views.whatilearened.today/views/github/aeonSolutions/aeonlabs-ESP32-C-Base-Firmware-Libraries.svg)
[![Open Source Love svg1](https://badges.frapsoft.com/os/v1/open-source.svg?v=103)](#)
[![contributions welcome](https://img.shields.io/badge/contributions-welcome-brightgreen.svg?style=flat&label=Contributions&colorA=red&colorB=black	)](#)
[<img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" data-canonical-src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" height="30" />](https://www.buymeacoffee.com/migueltomas)

[Open Software Caralog](https://github.com/aeonSolutions/aeonlabs-open-software-catalogue)  >>  APRS CWOP Library

<p align="right">
 <a href="https://github-com.translate.goog/aeonSolutions/aeonlabs-ESP32-C-Base-Firmware-Libraries?_x_tr_sl=en&_x_tr_tl=pt&_x_tr_hl=en&_x_tr_pto=wapp">Change Language</a> <br>
Last update: 27-12-2023
</p>

# [![CodeFactor](https://www.codefactor.io/repository/github/aeonsolutions/aeonlabs-aprs-cwop-library/badge)](https://www.codefactor.io/repository/github/aeonsolutions/aeonlabs-aprs-cwop-library) APRS CWOP Library for the ESP32
**current project Total: > 8 work.h**

This is a simple C++ class library I maintain to speed up firmware code development on all my smart electronics for weather stations reporting to the Citizen Weather Observer Program (CWOP) using APRS .
This is still an early code development. Is memory intensive and lacks many code optimizations.

The current code development is heavily based on the code by [cstroie](https://github.com/cstroie) found on this github repository: https://github.com/cstroie/WxUno

<br>

## Citizen Weather Observer Program
The [Citizen Weather Observer Program](https://www.weather.gov/cle/CWOP) (CWOP) is part of the USA's [national oceanic and atmospheric administration](https://www.noaa.gov) (NOAA) composed of a network of privately owned electronic weather stations concentrated in the United States and also in over 150 countries. The CWOP was originally set up by amateur radio operators experimenting with packet radio, but now contains a majority of Internet-only connected stations, with more than 10,000 stations worldwide report regularly to the CWOP network (July 2015). This network allows volunteers with computerized weather stations to send automated surface weather observations to the National Weather Service in the USA. This data is then used by the [Rapid Refresh forecast model](https://rapidrefresh.noaa.gov) to produce short term forecasts (3 to 12 hours into the future) of conditions across the United States' lower 48 states or in any other country. CWOP Observations are also re-distributed to the public. Before being used, there's an extensive set of quality control verifications, assigns a data quality rating and makes suggestions before is considered for modelling and forecasting the weather.

**How to connect to the CWOP Network** <br>
First one needs to signup at https://madis.ncep.noaa.gov/madis_cwop.shtml and when the token ID arrives it  needs to be added to the firmware code. An easy way to submit weather reports to the NOAA Citizen's Weather Observer Program is by using the [cwop.rest API](http://cwop.rest/) or the [Ambient Weather API](https://ambientweather.docs.apiary.io/#reference/ambient-realtime-api). Those looking into a vanilla C++ code implementation one can find it on GitHub on this repository and on [Costin Stroie's](https://github.com/cstroie) repository: https://github.com/cstroie/WxUno. To test if your hardware setup if is sending APRS packets and to ensure that everything is working properly one can look at a site like https://aprs.fi/ to see if it was received by the terrestrial network.

**Important links** <br>
- [HX1 APRS Transmitter Hookup Guide](https://learn.sparkfun.com/tutorials/hx1-aprs-transmitter-hookup-guide/all) by Sparkfun
- [APRS Specification](http://www.aprs.org/doc/APRS101.PDF)
- [Telemetry format and app notes](https://github.com/PhirePhly/aprs_notes/blob/master/telemetry_format.md) on Github
- [Sparkfun's Trackuino](https://github.com/sparkfun/SparkFun_Trackuino)
- [Arduino ARPS](https://handiko.github.io/Arduino-APRS/) by [handiko](https://github.com/handiko)
  
<br>

## How to use this Library
If you're using Arduino Studio for windows, download this repository and unzip it to the "Arduino/libraries" folder located in "Documents". <br>
Next rename the unziped folder "AeonLabs-APRS-CWOP-Library-main" to "AeonLabs_APRS_CWOP". <br>
Restart Arduino Studio, and the Aeonlabs library is now ready to be used. 


<br>

## Library Dependencies
- ✓ ESP32 C++ Base Firmware Libraries [view](https://github.com/aeonSolutions/aeonlabs-ESP32-C-Base-Firmware-Libraries#readme)

<br>

### Smart Devices I prototyped and using this code on their OEM firmware versions

- ✓ [Smart rain meter device](https://github.com/aeonSolutions/aeonlabs-HomeAutomation-Outdoors-Wireless-Battery-powered-Rainmeter)

<br>

### Useful utilities

[Arduino ESP8266/ESP32 Exception Stack Trace Decoder](https://github.com/me-no-dev/EspExceptionDecoder)

<br />
<br />

## Author

You can get in touch with me on my LinkedIn Profile:

#### Miguel Tomas

[![LinkedIn Link](https://img.shields.io/badge/Connect-Miguel--Tomas-blue.svg?logo=linkedin&longCache=true&style=social&label=Connect)](https://www.linkedin.com/in/migueltomas/)

<a href="https://stackexchange.com/users/18907312/miguel-silva"><img src="https://stackexchange.com/users/flair/18907312.png" width="208" height="58" alt="profile for Miguel Silva on Stack Exchange, a network of free, community-driven Q&amp;A sites" title="profile for Miguel Silva on Stack Exchange, a network of free, community-driven Q&amp;A sites" /></a>

<a href="https://app.userfeel.com/t/2f6cb1e0" target="_blank"><img src="https://app.userfeel.com/tester/737648/image?.png" width="257" class="no-b-lazy"></a>

You can also follow my GitHub Profile to stay updated about my latest projects: [![GitHub Follow](https://img.shields.io/badge/Connect-Miguel--Tomas-blue.svg?logo=Github&longCache=true&style=social&label=Follow)](https://github.com/aeonSolutions)

**Hire me** <br>
See [here](https://github.com/aeonSolutions/PCB-Prototyping-Catalogue/wiki/How-to-Hire-AeonLabs) how to hire AeonLabs.

### Be supportive of my dedication and work towards technology education and buy me a cup of coffee
The PCB design Files I provide here for anyone to use are free. If you like this Smart Device or use it, please consider buying me a cup of coffee, a slice of pizza or a book to help me study, eat and think new PCB design files.

<p align="center">
    <a href="https://www.buymeacoffee.com/migueltomas">
        <img height="35" src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png">
    </a>
</p>


### Make a donation on PayPal
Make a donation on PayPal and get a TAX refund*.

<p align="center">
    <a href="http://paypal.me/mtpsilva">
        <img height="35" src="https://github.com/aeonSolutions/PCB-Prototyping-Catalogue/blob/main/media/paypal_small.png">
    </a>
</p>

### Support all these open hardware projects and become a GitHub sponsor  
Liked any of my PCB KiCad Designs? Help and Support my open work to all by becoming a GitHub sponsor.

<p align="center">
    <a href="https://github.com/aeonSolutions/PCB-Prototyping-Catalogue/blob/main/become_a_sponsor/aeonlabs-github-sponsorship-agreement.docx">
        <img height="50" src="https://github.com/aeonSolutions/PCB-Prototyping-Catalogue/blob/main/media/want_to_become_a_sponsor.png">
    </a>
    <a href="https://github.com/sponsors/aeonSolutions">
        <img height="50" src="https://github.com/aeonSolutions/PCB-Prototyping-Catalogue/blob/main/media/become_a_github_sponsor.png">
    </a>
</p>

# 

### License

Before proceeding to download any of AeonLabs software solutions for open-source development and/or PCB hardware electronics development make sure you are choosing the right license for your project. See [AeonLabs Solutions for Open Hardware & Source Development](https://github.com/aeonSolutions/PCB-Prototyping-Catalogue/wiki/AeonLabs-Solutions-for-Open-Hardware-&-Source-Development) for more information. 

