## Components


|Category           |  #| What                                               |              | Supplier        |
|:------------------|--:|:---------------------------------------------------|:-------------|:----------------|
|3D Print           | 1 | ABS 3D printer                                     | <img width="100" src="https://store.zortrax.com/image/cache/catalog/new_carousel/M200_FI_U_02_C50_04-489x489.png"/> | [Zortrax M200](https://store.zortrax.com/M200) |
|                   |   | Petroleum, Acetone                                 | <img width="100" src="https://upload.wikimedia.org/wikipedia/commons/thumb/1/19/Acetone-structural.png/255px-Acetone-structural.png"/> | Local Dealer    | 																																			        |
|Electronics        | 1 | ODroid C2, 1.5 GHz Quadcore board                  | <img width="100" src="http://www.hardkernel.com/main/_Files/prdt/2016/201602/ODROID-C2.png"/>                       |  [ODroid C2](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G145457216438) |
|                   | 1 | Teensy 3.5, Arm M4 board                           | <img width="100" src="https://www.pjrc.com/store/teensy35.jpg"/>                                                    | [Teensy 3.5](https://www.pjrc.com/store/teensy35.html ) |
|3x Omniwheel       |  |                                                     |             |                  |
|                   |  | 2-component epoxy glue with at least 5 minutes set time (enough for one omniwheel) | <img width="100" src="https://images-na.ssl-images-amazon.com/images/I/41NpN3-Fl-L.jpg"/> | [Epoxy30](https://www.amazon.de/Minuten-Epoxy-Kleber-Transparent-100/dp/B00MKAW3DA/ref=sr_1_1?ie=UTF8&qid=1534629955&sr=8-1&keywords=epoxy+30) |
|                   |  | instant adhesive for porose surfaces                | <img width="100" src="http://hybris.cms.henkel.com/medias/sys_master/root/8799844728862/401-new.jpg"/> | [Loctite 401](http://www.loctite.de/produktsuche-29727.htm?nodeid=8802625323009) |

|                   |  |Corundum (sharp-edged), 5kg, 0.15-0.2mm grain size   |  <img width="100" src="https://upload.wikimedia.org/wikipedia/commons/thumb/6/63/Corundum-215245.jpg/600px-Corundum-215245.jpg"/> |[Corundum 0.15-0.2mm](https://www.ebay.de/itm/Strahlmittel-Glasperlen-Korund-Schlacke-Granatsand-Strahlgut-Sandstrahlen/172177067505?ssPageName=STRK%3AMEBIDX%3AIT&var=471011289734&_trksid=p2060353.m2749.l2649) | 
|                   |3x24|Ball bearing 2x5x2                                   | <img width="100" src="https://www.kugellager-express.de/media/image/product/6134/md/miniatur-kugellager-zoll-inch-r188-w3-175-offen-6-35x12-7x3-175-mm.jpg"/> | [Deep grove ball bearing 2x5x2](https://www.kugellager-express.de/miniatur-kugellager-682-zz-2x5x2-3-mm) |
|                   |3x1| Mounting Hub for 4mm axis                            | <img width="100" src="https://a.pololu-files.com/picture/0J1106.600x480.jpg?11d07bed4679844014f660800dd55548"/> | [Deep grove ball bearing 2x5x2](https://www.pololu.com/product/1997) |
|                   |3x4| Allen screws M3, 8mm length, DIN 912                 | <img width="100" src="https://upload.wikimedia.org/wikipedia/commons/thumb/0/07/Inbus-Schraube.jpg/440px-Inbus-Schraube.jpg"/> | local dealer |
|                   |3x6| AllenScrews M2, 8mm length, DIN 912                 | <img width="100" src="https://upload.wikimedia.org/wikipedia/commons/thumb/0/07/Inbus-Schraube.jpg/440px-Inbus-Schraube.jpg"/> | local dealer |
|                   |3x6| Screw Nut M2 DIN 934                                | <img width="100" src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcSp9dF9gi3EpbgLt_nPm75ovYIl1Juc83buPc0nQd0NHlBL4CLy"/> | local dealer |



## Sources

|Category              |  Description                                                        | Source     |
|:---------------------|:--------------------------------------------------------------------|:-----------|
|CAD Model             | Inventor model of all 3D parts                                      | [Github Ondine CAD](https://github.com/jochenalt/ondine/tree/master/CADInventor) |
|                      | 3D printable files of body and legs parts                           | [Github Ondine STL](https://github.com/jochenalt/ondine/tree/master/cad/stl) |
|Electronics           | Eagle Schematics and PCBs                                           | [Github Schematics](https://github.com/jochenalt/ondine/tree/master/schematics) |
|Source Code           | Arduino C++ code running on Teensy 3.5                              | [Github BotController](https://github.com/jochenalt/ondine/tree/master/code/BotController) |
|                      | Brain: ROS/Ubuntu C++ code running on Odroid C2, IDE Eclipse        | [Github ROS Nodes](https://github.com/jochenalt/ondine/tree/master/ros/src) |


## Tools
|Category              |  Description                                                        | Source     |
|:---------------------|:--------------------------------------------------------------------|:-----------|
|IDE for Teensy        | Eclipse derivate to develop and deploy in C++ on Teensy             |  [Eclipse Sloeber](http://eclipse.baeyens.it)                                      |
|ROS Kinetic           | ROS Version I used. Although the newer *lunar* is out for a while, some tools required (e.g. hector slam) are not yet available on Lunar. So, I went with the elder version |  [ROS Kinetic](http://wiki.ros.org/kinetic) |

