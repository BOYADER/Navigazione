# Navigazione Waypoint: Guida al codice
Implementazione di un Filtro di Kalman Esteso per la stima dello stato di un AUV.
La missione consiste nel raggiungere dei waypoints di cui le coordinate (Lat. Long. e Prof.) sono espresse nel file `mission.yaml`.

## Contenuti:
* [1. Requisiti](#1-requisiti)
* [2. Simulazione](#2-simulazione)
* [3. Struttura del pkg](#3-struttura-del-pkg-di-navigazione)
* [4. Nodi](#4-nodi)
* [5. File YAML](#5-file-yaml)
* [6. Files launch](#6-files-launch)

## 1) Requisiti
Per eseguire correttamente la simulazione ed ottenere i risultati mostrati nel report, è opportuno assicurarsi di avere:

- eigen3 nel path `/usr/include/eigen3`
- Avere scaricato tutti e 3 i pkgs del Team Waypoints. Altrimenti eseguire:
     ```
        ~/catkin_ws/src:
        git clone https://github.com/BOYADER/Navigazione
        git clone https://github.com/BOYADER/Pianificazione-Controllo
        git clone https://github.com/BOYADER/Modellazione 
     ```
- Rendere eseguibili i nodi python del pkg pc_wp:
     ```
        ~/catkin_ws/src/Pianificazione-Controllo/pc_wp/scripts ~$ chmod +x *
     ```  
- Scaricare pymap3d e termcolor:
  https://pypi.org/project/pymap3d/ ; https://pypi.org/project/termcolor/.

## 2) Simulazione
Per lanciare la simulazione, basta lanciare da terminale:
 ```
      ~$ roslaunch pc_wp launch.launch
 ```
Il launch appartiene al pkg pc_wp e lancia i nodi principali dei pkgs del Team. Per visualizzare meglio la simulazione, vengono inoltre lanciati dei nodi rqt_plot. 
(https://github.com/BOYADER/Pianificazione-Controllo/blob/main/pc_wp/launch/launch.launch)
E' possibile plottare:

- Stato vero e stato stimato.
- Forze e Coppie generate dal blocco di Controllo.
- Errore sulle singole grandezze.
- Errore quadratico Medio (MSE) calcolato iterativamente al variare del tempo.

![alt text](/rqt_graph.PNG)

## 3) Struttura del pkg di Navigazione

![alt text](/nav_pkg_screen.PNG)

## 4) Nodi

1. **EKF.cpp**: Nodo che sottoscrive i topics _/sensor_, esegue l’algoritmo
dell’Extended Kalman Filter e pubblica la stima dello stato nel topic _/odom_. In
realtà il nodo pubblica un altro topic, _/pos_ned_, utile solo per confrontare la
posizione del blocco di modellazione e la stima della posizione con _rqt_plot_.

2. **performance_analysis.cpp**: Nodo che sottoscrive i topics _/odom_ e _/state_real_,
calcola l’errore sulle variabili cinematiche ed infine l’MSE ad ogni iterazione.

3. **fake_modellazione.cpp**: Nodo di testing. Simula il blocco modellazione con dei dati costanti e senza rumore. 

4. **PF.cpp**: Nodo _incompleto_ che realizza un Filtro a Particelle con lo stesso scopo di _EKF.cpp_ . Il nodo presenza un'istruzione `break` che interrompe il nodo alla prima iterazione del ciclo `while(ros::ok())`.

## 5) File YAML
L’unico file YAML nel pkg si chiama `mission.yaml` e si trova al path:
`~/nav_pkg/config/mission.yaml` .

Il file contiene:
- Le coordinate geografiche della posa iniziale e dei waypoints da raggiungere
in missione.
- Valori delle deviazioni standard relative al disturbo di processo supposto nel
modello.
- Valori delle deviazioni standard relative al rumore di misura dei sensori
utilizzati.
- N° di misure valide necessarie per l’inizializzazione.

Il file inoltre contiene anche alcuni parametri aggiuntivi, necessari allo sviluppo del
Filtro a Particelle. (Numero di particelle e dev. std. sulla variabile di stato della vel. angolare).


## 6) Files launch
Sono state fatte numerose prove per l’integrazione (intermedia e finale), dunque
sono presenti vari file launch.
Per lanciare uno di questi launch, sul terminale basta digitare:

```
   roslaunch nav_pkg <nome_file>.launch
```  

1. `debug2.launch` e `performance2.launch`: Esegue una simulazione
in cui il blocco di controllo retroaziona _/state_real_, lasciando in catena aperta
il filtro. Questo permette di vedere gli ordini di grandezza delle velocità e
accelerazioni richieste dal blocco di Pianificazione e Controllo. Questo è stato
pensato per poter fare un tuning del filtro, evitando che l’azione di controllo
risenta delle inevitabili oscillazioni della stima. I due launch si differenziano da
ciò che mandano ai nodi rqt_plot. Il primo mostra infatti negli stessi grafici la
stima e lo stato vero del sistema. Il secondo invece grafica gli errori sulle
singole variabili di stato e l’MSE al variare del tempo.
Per eseguire ciò, vista l’esistenza di un blocco che pubblica /odom nel pkg
pc_wp, è necessario modificare nel file _EKF.cpp_ (riga 113) il nome del topic in
“`/odom_1`”.
     
2. `debug.launch` e `performance.launch`: Analogamente a quelli
precedenti, questi 2 files launch eseguono una simulazione senza il blocco di
controllo. Non è necessario in questo caso cambiare il nome del topic.

