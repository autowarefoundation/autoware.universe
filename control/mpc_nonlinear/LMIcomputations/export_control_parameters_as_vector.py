import yaml
import pickle
import numpy as np
from load_matlab_convert_pickle import load_params

np.set_printoptions(precision=6, suppress=True)

export_path = "export/"
option = 1

matfile_name = 'Lyaps_kinematic_control_steering_delay'
xtag = "n"
load_params(matfile_name)

with open(matfile_name + '.pickle', 'rb') as handle:
    systemMats = pickle.load(handle)

if __name__ == '__main__':

    fname = export_path + matfile_name + '.yaml'
    ff = open(fname, 'w')

    # Last item in the dictionary is X0, Y0
    Xs = systemMats['Xs'].flatten(order='F').tolist()
    Ys = systemMats['Ys'].flatten(order='F').tolist()

    for i in range(len(Xs)):
        text = f'X{xtag}{i + 1}: ' + '{}'.format(Xs[i].flatten(order='F').tolist()) + "\n"
        # print(text)
        ff.write(text)

        # yaml.dump(text, ff)
        print(f'\nX_{i} : \n', Xs[i])

    for i in range(len(Ys)):
        text = f'Y{xtag}{i + 1}: ' + '{}'.format(Ys[i].flatten(order='F').T.tolist()) + "\n"

        # print(text)
        ff.write(text)

        print(f'\nY_{i} :\n', Ys[i])

    # for k in range(5):
    #     xs = Xs[k]
    #     print(xs)

    # print("numpy array ")
    # print(np.array(Xs[4].flatten(order='F').T.tolist()).reshape(-1, 4))
    a = 1
