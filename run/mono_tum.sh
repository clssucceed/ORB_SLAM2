SEQUENCE_NUMBER=${1}
PATH_TO_SEQUENCE_FOLDER=${2}
cd ..
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM${SEQUENCE_NUMBER}.yaml ${PATH_TO_SEQUENCE_FOLDER} 
