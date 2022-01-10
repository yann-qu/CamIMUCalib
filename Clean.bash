echo "输入1或2选择要清理的数据"
echo "1. Clean CamCalib data"
echo "2. Clean CamIMUCalib Data"

read Option

if [ $Option = 1 ]; then
  DIR="CamCalib/img"

  # look for empty dir
  if [ "$(ls -A $DIR)" ]; then
      echo "$DIR is not Empty"
      rm $DIR/
      echo "Clean success"
  else
      echo "$DIR is Empty"
  fi
fi

if [ $Option = 2 ]; then
  DIR="data"

  # look for empty dir
  if [ "$(ls -A $DIR)" ]; then
      echo "$DIR is not Empty"
      rm -r $DIR/*
      echo "Clean success"
      mkdir $DIR/img $DIR/IMU
  else
      echo "$DIR is Empty"
  fi
fi

#rm data/img/*
#rm data/IMU/*
