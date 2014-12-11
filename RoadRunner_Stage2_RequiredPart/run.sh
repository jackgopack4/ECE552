rm tars_dir/RR.tar.gz
rm -rf RR.tar.gz_extracted/
cd ..
tar -cvzf RoadRunner_Stage2_RequiredPart/tars_dir/RR.tar.gz *.v common_params.inc
cd RoadRunner_Stage2_RequiredPart/
python RoadRunner.py --sim=vvp
