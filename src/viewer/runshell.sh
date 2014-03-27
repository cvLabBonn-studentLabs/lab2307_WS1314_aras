#!/bin/sh
#
#

export IFS=";"

path_prefix="/home/eastfold/data/reOrganize/JuergenDataset/table-object"
write_prefix="/home/eastfold/data/reOrganize/JuergenDataset/processed_data/table-object"
write_extn="/joints_rgb/"


dep_file="joints_dep"
rgb_file="joints_rgb_un"

start_idx=(90 50 40 50 40 45 55 50 40 45 40 35 40 40 40 50 35 50 50 60 50 40 40 40 50 40 35)
end_idx=(430 450 370 205 175 285 335 300 160 175 170 155 175 185 155 145 210 240 230 290 280 345 375 235 260 190 205)
object_num="11"
serial_num=("002" "003" "005" "006" "007" "008" "009" "010" "011" "012" "013" "014" "015" "016" "017" "018" "019" "020" "021" "022" "023" "024" "026" "027" "028" "030" "031")

:<<'WORLD_END'
start_idx=(30 35)
end_idx=(200 220)
object_num="12"
serial_num=("000" "001")

start_idx=(70)
end_idx=(510)
object_num="08"
serial_num=("000")

start_idx=(50 45 40 50 60 40 40 45 40 40 40 50 35 40 50 50 50 50 40 40 50 50 45 45)    
end_idx=(395 350 355 180 180 275 270 250 165 140 150 160 180 190 165 150 165 160 195 185 320 300 230 260)   
object_num="10"
serial_num=("001" "002" "003" "004" "005" "006" "007" "008" "009" "010" "011" "012" "013" "014" "015" "016" "017" "018" "019" "020" "021" "022" "023" "024") 

start_idx=(55 40 35 30 40 40 40 40 40 35 35 15 35 30 40 35 50 35 40 30 35 35 40 40 40 30)
end_idx=(310 310 320 295 155 155 210 175 185 155 160 150 135 145 165 140 195 180 185 150 285 260 235 195 170 175)
object_num="13"
serial_num=("002" "003" "004" "005" "006" "007" "008" "009" "010" "011" "012" "013" "014" "015" "016" "018" "019" "020" "021" "022" "023" "024" "025" "026" "027" "028")

start_idx=(60 55 55 50 50 90 50 50 50 55 40 40 35 40 35 40 30 35 40 40 45 35 40 35 35)
end_idx=(390 340 195 160 200 260 220 170 155 170 160 160 155 125 130 205 175 185 170 325 330 210 175 170 180)
object_num="14"
serial_num=("001" "003" "005" "006" "007" "008" "009" "010" "011" "012" "013" "014" "015" "017" "018" "019" "020" "021" "022" "023" "024" "025" "026" "027" "028")

start_idx=(40 55 30 30 40 40 35 30 25 35 30 30 25 30 30 30 25 30 50 40)
end_idx=(400 180 165 245 275 225 145 145 120 125 155 135 190 175 255 330 240 270 200 185)
object_num="16"
serial_num=("003" "006" "007" "008" "009" "010" "011" "012" "013" "014" "015" "020" "021" "022" "023" "024" "025" "026" "027" "028")

start_idx=(75 50 50 55 45 40 40 35 40 30 30 35 35 35 35 35 40 40 40 40 35 35 40 40 30 45 45)
end_idx=(420 425 390 425 185 155 185 230 180 130 135 130 140 170 160 160 160 140 140 180 170 280 280 210 205 220 215)
object_num="17"
serial_num=("001" "002" "003" "004" "005" "006" "007" "008" "009" "010" "011" "012" "013" "014" "015" "016" "017" "018" "019" "020" "021" "022" "023" "024" "025" "026" "027")
  
WORLD_END

for idx in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26
do
  rm tempconfig.txt
  path=$path_prefix$object_num"/data/s"${serial_num[$idx]}".bmf"
  echo $path >> tempconfig.txt
  path=$path_prefix$object_num
  echo $path >> tempconfig.txt
  echo ${start_idx[$idx]} >> tempconfig.txt
  echo ${end_idx[$idx]} >> tempconfig.txt
  path=$path_prefix$object_num"/data/calib_TOF.txt"
  echo $path >> tempconfig.txt
  path="calib_RGB.txt"
  echo $path >> tempconfig.txt
  path=$path_prefix$object_num"/data/model.dat"
  echo $path >> tempconfig.txt
  path=$path_prefix$object_num"/out"${serial_num[$idx]}"_1"
  echo $path >> tempconfig.txt
  path=$write_prefix${object_num[$idx]}$write_extn
  echo $path >> tempconfig.txt
  
  ./Project "tempconfig.txt" "./tempOutput"
  src_file="./tempOutput/"$dep_file".txt"
  dst_file="./tempOutput/"$dep_file"_"${serial_num[$idx]}".txt"
  echo $src_file
  mv $src_file $dst_file
  src_file="./tempOutput/"$rgb_file".txt"
  dst_file="./tempOutput/"$rgb_file"_"${serial_num[$idx]}".txt"
  mv $src_file $dst_file
  sleep 3
done

