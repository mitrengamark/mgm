# Gyak11
Készítsen egy package-t, mely bejárt útvonalak lementésére szolgál egy fájlba, valamint képes azt beolvasva útvonalként kikommunikálni.

## Node1 save_path
Készítsen egy node-t, mely be kér paraméterként egy topic nevet, amire felíratkozva nav_msgs/Odometry típusú  üzeneteket tud fogadni. Az Odometry üzeneteket mentse el egy fájlba, melynek elérési útját paraméterként kérje be. (3 pont)
A mentés során ellenőrizze, hogy az előző elmentett ponttól minimum egy adott távolságra, a távolságot szintén paraméterként kérje be. (2 pont)
A mentendő adatok, "map" frame-ben legyenek értelmezve:
- x, y, z méterben,
- yaw fokban értelmezve. (3 pont)
Készítsen hozzá launch fájlt, a megfelelő paraméterekkel.(1 pont)

## Node2 pub_path
Készítsen egy node-t, mely be kér egy elérési utat paraméterként, ahol egy felvett útvonal található. A benne lévő adatok mennyisége és minősége megegyezik az előző  feladatban foglaltakkal. Olvassa be a fájlt és a benne lévő pálya információt küldje ki a "/path" topicra nav_msgs/Path típusú üzenetként 1 Hz-es időzítéssel. (4 pont)
Kérje be paraméterként, hogy milyen frame-ban legyen értelmezve az adat és konvertálja át arra.(3 pont)
Keresse ki a minimális ás maximális értékeit az x és y koordinátáknak és küldjön ki egy visualization_msgs/MarkerArray típusú üzenetet a "/viz" topicra, melyben két 10 cm átmérőjű gömböt helyez el a (x_min, y_min) és a (x_max, y_max) koordinátákra piros színnel.(4 pont)
Készítsen hozzá launch fájlt, a megfelelő paraméterekkel.(1 pont)

Tesztelésre használhatja a gyak3-ban lévő bag-fájlt.
