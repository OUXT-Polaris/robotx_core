for package in robotx_msgs
do
    roscd $package
    bloom-generate rosdebian --os-name ubuntu --os-version xenial --ros-distro kinetic
    fakeroot debian/rules binary
    rm -rf debian
    rm -rf obj-x86_64-linux-gnu
done

cd ../
mv *.deb ./build/amd64