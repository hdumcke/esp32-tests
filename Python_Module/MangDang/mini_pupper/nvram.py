import pickle

ServoCalibrationFilePath = '/home/ubuntu/nvmem'


def write(data):
    with open(ServoCalibrationFilePath, 'wb') as fd:
        pickle.dump(data, fd, protocol=pickle.HIGHEST_PROTOCOL)


def read():
    with open(ServoCalibrationFilePath, 'rb') as fd:
        return pickle.load(fd)
