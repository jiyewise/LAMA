echo "installing python env"

python3 -m venv py_env
. ./py_env/bin/activate
pip3 install --upgrade pip
pip3 install -r requirements.txt
# pip install torch==1.10.1+cu113 torchvision==0.11.2+cu113 torchaudio==0.10.1+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html
pip3 install torch==1.13.0 torchvision==0.14.0 --index-url https://download.pytorch.org/whl/cu117
cd fairmotion
pip3 install -e .
cd ..