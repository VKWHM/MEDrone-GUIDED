sudo apt-get update;
sudo apt-get upgrade;
sudo apt-get install python3-matplotlib python3-serial python3-wxgtk4.0 python3-lxml python3-scipy python3-opencv ccache gawk python3-pip python3-pexpect;
pip3 install -r requirements.txt
pip3 install opencv-python
sed -i 's/collections/collections.abc/g' ~/.local/lib/python$(python3 --version | awk '{ split($2, v, "."); print sprintf("%d.%d", v[1], v[2]) }')/site-packages/dronekit/__init__.py

