if [ -d "./CodeCraft-2019" ]; then
	rm -r CodeCraft-2019
fi
mkdir CodeCraft-2019
cp CMakeLists.txt ./CodeCraft-2019
cp *.cpp ./CodeCraft-2019
cp *.h ./CodeCraft-2019
mv CodeCraft-2019/HCC2019.cpp CodeCraft-2019/CodeCraft-2019.cpp
