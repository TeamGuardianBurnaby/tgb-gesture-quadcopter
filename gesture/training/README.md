https://www.pyimagesearch.com/2018/04/16/keras-and-convolutional-neural-networks-cnns/

https://stackoverflow.com/questions/38896424/tensorflow-not-found-using-pip

Install anacouda.

https://docs.anaconda.com/anaconda/user-guide/tasks/tensorflow/

# Training Data
To train the network, run the following command inside Anaconda Powershell:

conda activate tf-gpu

python train-gesture.py --dataset dataset --model gesture.model --labelbin lb.pickle

python classify.py --model gesture.model --labelbin lb.pickle --image example.jpg

python gesture-control.py --model gesture.model --labelbin lb.pickle
