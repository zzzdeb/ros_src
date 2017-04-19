sudo mount //imr-fs03.imr.rwth-aachen.de/Messdaten -o username=zerdenebayar,domain=win-imr,uid=1000,gid=1000 -t cifs /media/messdaten
cd /media/messdaten/SuA/2017-02-07-Reichezeche_FH_all_data/2017.02.07_Reichezeche/
rosbag play [s,d]*-13-1*.bag --clock -l
