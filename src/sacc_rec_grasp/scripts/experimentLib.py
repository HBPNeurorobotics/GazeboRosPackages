#!/bin/python
# -----------------------------------------------------------------------------
# File Name : experimentLib.py
# Purpose:
#
# Author: Emre Neftci
#
# Creation Date : 31-03-2015
# Last Modified : Mon 10 Apr 2017 08:53:21 PM PDT
#
# Copyright : (c)
# Licence : GPLv2
# -----------------------------------------------------------------------------
#
### Stand-alone functions only! no custom dependence
import numpy as np
import sys, os, glob
import pandas as pd


def pandas_loadtxt_2d(f, delimiter=' ', *args, **kwargs):
    try:
        out = pd.read_csv(f, *args, delimiter=delimiter, **kwargs).values[:, [0, 1]]
        return out
    except pd.io.common.EmptyDataError, e:
        # print("pandas: Empty Data")
        return np.zeros([0, 2])


def dense_to_one_hot(labels_dense, num_classes=10):
    """Convert class labels from scalars to one-hot vectors."""
    num_labels = labels_dense.shape[0]
    index_offset = np.arange(num_labels) * num_classes
    labels_one_hot = np.zeros((num_labels, num_classes))
    labels_one_hot.flat[index_offset + labels_dense.ravel()] = 1
    return labels_one_hot


def compute_synops_rbm(N, Nv=794, Nh=500, Connhv=None, Connvh=None):
    if Connhv is None:
        Connhv = np.ones([Nh, Nv])
    else:
        Connhv = np.array(Connhv)
    fanout_h = Connhv.sum(axis=1).astype('int')

    if Connvh is None:
        Connvh = np.ones([Nv, Nh])
    else:
        Connvh = np.array(Connvh)
    fanout_v = Connvh.sum(axis=1).astype('int')

    synops = 0
    for i in range(Nh):
        synops += fanout_h[i] * N['h'][i, 1]
    for i in range(Nv):
        synops += fanout_v[i] * N['v'][i, 1]
    return synops


class DataSet2D(object):
    def __init__(self, images, labels, fake_data=False):
        if fake_data:
            self._num_examples = 10000
        else:
            assert images.shape[0] == labels.shape[0], (
                    "images.shape: %s labels.shape: %s" % (images.shape, labels.shape))
            self._num_examples = images.shape[0]

            # Convert shape from [num examples, rows, columns, depth]
            # to [num examples, rows*columns] (assuming depth == 1)
            assert images.shape[3] == 1
            images = images.reshape(images.shape[0], images.shape[1] * images.shape[2])
            # Convert from [0, 255] -> [0.0, 1.0].
            images = images.astype(np.float32)
            images = np.multiply(images, 1.0 / 255.0)
        self._images = images
        self._labels = dense_to_one_hot(labels, num_classes=10)
        self._epochs_completed = 0
        self._index_in_epoch = 0

    @property
    def images(self):
        return self._images

    @property
    def labels(self):
        return self._labels

    @property
    def num_examples(self):
        return self._num_examples

    @property
    def epochs_completed(self):
        return self._epochs_completed

    def next_batch(self, batch_size, fake_data=False):
        """Return the next `batch_size` examples from this data set."""
        if fake_data:
            fake_image = [1.0 for _ in xrange(784)]
            fake_label = 0
            return [fake_image for _ in xrange(batch_size)], [
                fake_label for _ in xrange(batch_size)]
        start = self._index_in_epoch
        self._index_in_epoch += batch_size
        if self._index_in_epoch > self._num_examples:
            # Finished epoch
            self._epochs_completed += 1
            # Shuffle the data
            perm = np.arange(self._num_examples)
            np.random.shuffle(perm)
            self._images = self._images[perm]
            self._labels = self._labels[perm]
            # Start next epoch
            start = 0
            self._index_in_epoch = batch_size
            assert batch_size <= self._num_examples
        end = self._index_in_epoch
        return self._images[start:end], self._labels[start:end]


def numpy_version_largerthan(version):
    return int(np.version.version.split('.')[1]) >= int(version.split('.')[1])


def textannotate(filename='', text=''):
    "Create a file, with contents text"
    f = file(filename, 'w')
    f.write(text)
    f.close()


def read_file(filename, **context):
    os.system('../tools/aubs -i outputs/{directory}/test/{0} -o /tmp/h'.format(filename, **context))
    return np.loadtxt('/tmp/h')


def clamped_input_transform_log(input_vector, min_p=1e-7, max_p=0.999, binary=False):
    '''
    Transforms the input vectors according to inverse sigmoid
    binary: inputs are binarized
    min_p: minimum firing probability. Should be >0
    max_p: minimum firing probability. Should be <1
    '''
    s = np.array(input_vector)  # Divide by t_ref to get firing rates
    if not binary:
        max_p_ = max_p
        min_p_ = min_p
    else:
        max_p_ = 0.5
        min_p_ = 0.5
    s[s < min_p_] = min_p
    s[s >= max_p_] = max_p
    s = -np.log(-1 + 1. / (s))
    return s


def select_equal_n_labels(n, data, labels, classes, seed=None):
    n_classes = len(classes)
    n_s = int(np.ceil(float(n) / n_classes))
    max_i = [np.nonzero(labels == i)[0] for i in classes]
    if seed is not None:
        np.random.seed(seed)
    f = lambda x, n: np.random.random_integers(0, x - 1, int(n))
    a = np.concatenate([max_i[i][f(len(max_i[i]), n_s)] for i in classes])
    np.random.shuffle(a)
    iv_seq = data[a]
    iv_l_seq = labels[a]
    return iv_seq, iv_l_seq


def load_mnist(data_url, labels_url):
    f_image = file(data_url, 'r')
    f_label = file(labels_url, 'r')
    # Extracting images
    m, Nimages, dimx, dimy = np.fromstring(f_image.read(16), dtype='>i')
    nbyte_per_image = dimx * dimy
    images = np.fromstring(f_image.read(Nimages * nbyte_per_image), dtype='uint8').reshape(Nimages,
                                                                                           nbyte_per_image).astype(
        'float') / 256

    # Extracting labels
    np.fromstring(f_label.read(8), dtype='>i')  # header unused
    labels = np.fromstring(f_label.read(Nimages), dtype='uint8')
    f_image.close()
    f_label.close()
    return images, labels


def load_data_labels(data_url, labels_url, n_samples=1, randomize=False, nc_perlabel=1, min_p=0.0001, max_p=.95,
                     binary=False, seed=None, nc=10, skip=None, limit=None, **kwargs):
    '''
    Loads MNIST data. Returns randomized samples as pairs [data vectors, data labels]
    test: use test data set. If true, the first n_sample samples are used (no randomness)
    nc: number of class units per label
    Outputs input vector, label vector and sequence of labels.
    kwargs unsed
    '''
    import gzip, cPickle

    iv, iv_l = load_mnist(data_url, labels_url)

    iv = iv[skip:limit]
    iv_l = iv_l[skip:limit]

    if randomize is False:
        # Do not randomize order of test in any case
        iv_seq, iv_l_seq = iv[:n_samples], iv_l[:n_samples]
    elif randomize == 'within':
        idx = range(n_samples)
        iv_seq, iv_l_seq = iv[:n_samples], iv_l[:n_samples]
        np.random.shuffle(idx)
        iv_seq = iv[idx]
        iv_l_seq = iv_l[idx]
    else:
        # Do randomize order of training
        iv_seq, iv_l_seq = select_equal_n_labels(n_samples, iv, iv_l, seed=seed, classes=range(nc))

    # expand labels
    if nc_perlabel > 0:
        iv_label_seq = np.zeros([n_samples, nc_perlabel * nc]) + min_p
        for i in range(len(iv_l_seq)):
            s = iv_l_seq[i] * nc_perlabel
            iv_label_seq[i, s:(s + nc_perlabel)] = max_p
    else:
        iv_label_seq = np.zeros([n_samples, 0], dtype='int')

    iv_label_seq = iv_label_seq
    return iv_seq, iv_label_seq, iv_l_seq


def Wround(W, wmin, wmax, wlevels, random=True):
    s = np.sign(W);
    wresol = float(wmax - wmin) / wlevels
    abseps = np.abs(W) / wresol;
    p = abseps - np.floor(abseps);
    Wr = np.zeros_like(W)
    if random:
        x = p > np.random.rand(*W.shape)
    else:
        x = p > .5
    if np.any(x):
        Wr[x] = s[x] * wresol * np.ceil(abseps[x])
    if np.any(~x):
        Wr[~x] = s[~x] * wresol * np.floor(abseps[~x])
    Wr[Wr < wmin] = wmin
    Wr[Wr > (wmax - wresol)] = (wmax - wresol)
    return Wr


def init_rbp1h_parameters(nv, nc, nh, mean_weight=0., std_weight=0.2, seed=0, rr=None, **kwargs):
    '''
    Initialize feed-forward deep neural network for random back-propagation parameters with 1 hidden layer
    nv: number of visible neurons
    nc: number of output neurons
    nh: number of hidden neurons
    '''
    np.random.seed(int(seed))
    avh = np.sqrt(std_weight / (nv + nh))
    Wvh = np.random.uniform(low=-avh, high=+avh, size=(nv, nh))
    Wvh[(nv - nc):, :] = 0
    CWvh = np.ones((nv, nh), dtype='bool')
    CWvh[(nv - nc):, :] = False
    Whh = np.zeros((nh, nh))
    CWhh = np.zeros((nh, nh), dtype='bool')
    aho = np.sqrt(std_weight / (nc + nh))
    Who = np.random.uniform(low=-aho, high=aho, size=(nh, nc))
    CWho = np.ones((nh, nc), dtype='bool')
    Woe = np.eye(nc) * -90e-3
    CWoe = np.eye(nc, dtype='bool')
    Wve = np.zeros((nv, nc))
    Wve[nv - nc:, :] = np.eye(nc, dtype='bool') * 90e-3
    CWve = Wve != 0
    Weo = np.eye(nc) * 90e-3
    CWeo = np.eye(nc, dtype='bool')
    aeh = np.sqrt(std_weight / (nc + nh))
    Weh = np.random.uniform(low=-aeh, high=aeh, size=(nc, nh))
    B = np.dot(Weh.T, np.ones(nc))
    Weh = Weh - B / nc
    CWeh = np.ones((nc, nh), dtype='bool')
    if rr == None:
        return {'vh': [Wvh, CWvh],
                'hh': [Whh, CWhh],
                'ho': [Who, CWho],
                'oe': [Woe, CWoe],
                've': [Wve, CWve],
                'eo': [Weo, CWeo],
                'eh': [Weh, CWeh]}
    else:
        return {'vh': [Wround(Wvh, random=True, **rr), CWvh],
                'hh': [Wround(Whh, random=True, **rr), CWhh],
                'ho': [Wround(Who, random=True, **rr), CWho],
                'oe': [Woe, CWoe],
                've': [Wve, CWve],
                'eo': [Weo, CWeo],
                'eh': [Wround(Weh, random=False, **rr), CWeh]}


def conv2d(imsize=28, ksize=5, stride=2):
    kx = ksize
    ky = ksize

    sx = imsize
    sy = imsize

    ox = imsize / stride
    oy = imsize / stride

    Kidx = np.arange(0, kx * ky, dtype='int').reshape(kx, ky)
    K = np.zeros([kx, ky])
    W = np.zeros([sx, sy, sx, sy], dtype='int') - 1

    for k in range(sx):
        for l in range(sx):
            for i in range(sx):
                for j in range(sy):
                    dx = k - i
                    dy = l - j
                    if dx in range(kx) and dy in range(ky):
                        W[k, l, i, j] = Kidx[dx, dy]
    Wd = W[:, :, ::stride, ::stride].reshape(sx * sy, ox * oy)
    return Wd, Wd != -1, K


def SLrates(filename, nx=28, ny=28):
    SL = monitor_to_spikelist(filename).id_slice(range(nx * ny))
    ssl = SL.time_slice(0, 250)
    ssl.complete(range(nx * ny))
    return ssl.mean_rates().reshape(nx, ny)


def init_cnn2L_parameters(nv, nc, nfeat1, nfeat2, nh, mean_weight=0., std_weight=0.2, seed=0, **kwargs):
    '''
    Initialize feed-forward deep neural network for random back-propagation parameters with 1 hidden layer
    nv: number of visible neurons
    nc: number of output neurons
    nh: number of hidden neurons
    '''
    np.random.seed(int(seed))
    nc1 = (nv - nc) / 4
    nc2 = (nv - nc) / 16
    Wvh = [None] * nfeat2
    CWvh = [None] * nfeat2
    avh = np.sqrt(std_weight / (nfeat2 * nc2 + nh))
    for i in range(nfeat2):
        Wvh[i] = np.random.uniform(low=-avh, high=+avh, size=(nc2, nh))
        CWvh[i] = np.ones((nc2, nh), dtype='bool')
    aho = np.sqrt(std_weight / (nc + nh))
    Who = np.random.uniform(low=-aho, high=aho, size=(nh, nc))
    CWho = np.ones((nh, nc), dtype='bool')
    Woe = np.eye(nc) * -90e-3
    CWoe = np.eye(nc, dtype='bool')
    Wve = np.zeros((nv, nc))
    Wve[nv - nc:, :] = np.eye(nc, dtype='bool') * 90e-3
    CWve = Wve != 0
    Weo = np.eye(nc) * 90e-3
    CWeo = np.eye(nc, dtype='bool')
    aeh = np.sqrt(std_weight / (nc + nh))
    Weh = np.random.uniform(low=-aeh, high=aeh, size=(nc, nh))
    B = np.dot(Weh.T, np.ones(nc))
    Weh = Weh - B / nc
    CWeh = np.ones((nc, nh), dtype='bool')

    Wec1 = [None] * nfeat1
    CWec1 = [None] * nfeat1
    aec1 = np.sqrt(std_weight / (nh + nc))
    for i in range(nfeat1):
        v = np.random.uniform(low=-aec1, high=aec1, size=(nc, nc1))
        B = np.dot(v.T, np.ones(nc))
        v = v - B / nc
        Wec1[i] = v / 10
        CWec1[i] = np.ones((nc, nc1), dtype='bool')

    Wec2 = [None] * nfeat2
    CWec2 = [None] * nfeat2
    aec2 = np.sqrt(std_weight / (nh + nc))
    for i in range(nfeat2):
        v = np.random.uniform(low=-aec2, high=aec2, size=(nc, nc2))
        B = np.dot(v.T, np.ones(nc))
        v = v - B / nc
        Wec2[i] = v / 10
        CWec2[i] = np.ones((nc, nc2), dtype='bool')

    Wc1, CWc1, K = conv2d(28, 5, 2)
    Wc2, CWc2, K = conv2d(14, 5, 2)

    ret_dict = {
        'ho': [Who, CWho],
        'oe': [Woe, CWoe],
        've': [Wve, CWve],
        'eo': [Weo, CWeo],
        'eh': [Weh, CWeh],
    }
    for i in range(nfeat2):
        ret_dict['vh_{0}'.format(i)] = [Wvh[i], CWvh[i]]
        ret_dict['ec2_{0}'.format(i)] = [Wec2[i], CWec2[i]]

    for i in range(nfeat1):
        ret_dict['ec1_{0}'.format(i)] = [Wec1[i], CWec1[i]]
    ret_dict['c1'] = [Wc1, CWc1] + [.1 * (np.random.uniform(size=[5, 5]) - .25) for i in range(nfeat1)]
    ret_dict['c2'] = [Wc2, CWc2] + [.1 * (np.random.uniform(size=[5, 5]) - .25) for i in range(nfeat1 * nfeat2)]
    return ret_dict


def init_rbp2h_parameters(nv, nc, nh1, nh2, mean_weight=0., std_weight=0.2, seed=0, recurrent=False, **kwargs):
    '''
    Initialize feed-forward deep neural network for random back-propagation parameters with 1 hidden layer
    nv: number of visible neurons
    nc: number of output neurons
    nh: number of hidden neurons
    '''
    print std_weight, nc, nh1, nh2, recurrent
    nh = nh1 + nh2
    np.random.seed(seed)
    avh = np.sqrt(std_weight / (nv + nh1))
    Wvh = np.zeros((nv, nh))
    Wvh[:, :nh1] = np.random.uniform(low=-avh, high=+avh, size=(nv, nh1))
    Wvh[(nv - nc):, :] = 0
    CWvh = np.zeros((nv, nh), dtype='bool')
    CWvh[:nv, :nh1] = True
    CWvh[(nv - nc):, :] = False
    Whh = np.zeros((nh, nh))
    CWhh = np.zeros((nh, nh), dtype='bool')
    ahh1 = np.sqrt(std_weight / (nh))
    Whh[:nh1, nh1:nh1 + nh2] = np.random.uniform(low=-ahh1, high=+ahh1, size=(nh1, nh2))
    CWhh[:nh1, nh1:nh1 + nh2] = True
    if recurrent:
        part = 5
        Whh[nh1:nh1 + int(nh2 / part), :int(nh1 / part)] = np.random.uniform(low=-ahh1, high=+ahh1,
                                                                             size=(int(nh1 / part), int(nh2 / part)))
        CWhh[nh1:nh1 + int(nh2 / part), :int(nh1 / part)] = True

    Wvh = np.random.uniform(low=-avh, high=+avh, size=(nv, nh1))
    Wvh[(nv - nc):, :] = 0

    aho = np.sqrt(std_weight / (nc + nh2))
    Who = np.zeros((nh, nc))
    CWho = np.zeros((nh, nc), dtype='bool')
    Who[nh1:, :] = np.random.uniform(low=-aho, high=aho, size=(nh2, nc))
    CWho[nh1:, :] = np.ones((nh2, nc), dtype='bool')
    Woe = np.eye(nc) * -90e-3
    CWoe = np.eye(nc, dtype='bool')
    Wve = np.zeros((nv, nc))
    Wve[nv - nc:, :] = np.eye(nc, dtype='bool') * 90e-3
    CWve = Wve != 0
    Weo = np.eye(nc) * 90e-3
    CWeo = np.eye(nc, dtype='bool')
    aeh = np.sqrt(std_weight / (nc + nh))
    Weh = np.random.uniform(low=-aeh, high=aeh, size=(nc, nh))
    B = np.dot(Weh.T, np.ones(nc))
    Weh = Weh - B / nc
    CWeh = np.ones((nc, nh), dtype='bool')

    return {'vh': [Wvh, CWvh],
            'hh': [Whh, CWhh],
            'ho': [Who, CWho],
            'oe': [Woe, CWoe],
            've': [Wve, CWve],
            'eo': [Weo, CWeo],
            'eh': [Weh, CWeh]}


def create_rbp_init(base_filename='fwmat', **kwargs):
    '''
    Create initial weight and bias parameters for the RBM.
    *beta_prm*:  beta parameter as defined in Neftci et al 2014
    *tau_rec*: time constant of the recurrent synapses, used for synapse based bias implementation (note: synapses  based biases are deprecated, newer versions use an adaptation term at the neuron level.
    *nv* and *nh*: number of visible and hidden neurons respectively
    *kwargs*: passed to init_rbm_parameters (define mean and std here)
    '''
    if kwargs.has_key('nh2'):
        W_CW = init_rbp2h_parameters(**kwargs)
    else:
        W_CW = init_rbp1h_parameters(**kwargs)
    for k in ['vh', 'hh', 'ho', 'oe', 've', 'eo', 'eh']:
        W, CW = W_CW[k]
        save_auryn_wmat('{0}_{1}.mtx'.format(base_filename, k), W, mask=CW)
    return W_CW


def to_auryn_conv2d(filename, Wd, K, n_K):
    ident = filename.split('fwmat_')[1]
    M = {}
    M[ident + '_cw'] = to_auryn_wmat(filename + '_cw', Wd, mask=Wd != -1)
    if len(n_K) == 1:
        for i in range(n_K[0]):
            k = '_w_{0}'.format(i)
            M[ident + k] = to_auryn_wmat(filename + k, K[i].reshape(-1, 1))
    elif len(n_K) == 2:
        for i in range(n_K[0]):
            for j in range(n_K[1]):
                k = '_w_{0}_{1}'.format(i, j)
                M[ident + k] = to_auryn_wmat(filename + k, K[j + i * n_K[0]].reshape(-1, 1))
    return M


def create_cnn_init(base_filename='fwmat', **kwargs):
    '''
    Create initial weight and bias parameters for the eRBP convnet.
    '''
    nfeat1 = kwargs['nfeat1']
    nfeat2 = kwargs['nfeat2']
    W_CW = init_cnn2L_parameters(**kwargs)
    M = {}
    for k in ['ho', 'oe', 've', 'eo', 'eh']:
        W, CW = W_CW[k]
        M[k] = to_auryn_wmat('{0}_{1}.mtx'.format(base_filename, k), W, mask=CW)

    M1 = to_auryn_conv2d("{0}_c1".format(base_filename), W_CW['c1'][0], W_CW['c1'][2:], [nfeat1])
    M2 = to_auryn_conv2d("{0}_c2".format(base_filename), W_CW['c2'][0], W_CW['c2'][2:], [nfeat1, nfeat2])

    for k in ['vh_{0}'.format(i) for i in range(nfeat2)]:
        W, CW = W_CW[k]
        M[k] = to_auryn_wmat('{0}_{1}.mtx'.format(base_filename, k), W, mask=CW)

    for k in ['ec1_{0}'.format(i) for i in range(nfeat1)]:
        W, CW = W_CW[k]
        M[k] = to_auryn_wmat('{0}_{1}.mtx'.format(base_filename, k), W, mask=CW)

    for k in ['ec2_{0}'.format(i) for i in range(nfeat2)]:
        W, CW = W_CW[k]
        M[k] = to_auryn_wmat('{0}_{1}.mtx'.format(base_filename, k), W, mask=CW)

    M.update(M1)
    M.update(M2)
    save_all_fwmat(M, '{0}_'.format(base_filename))
    return M


##### from create_visible_data.py ###

def create_tiser(input_vectors, duration, pause, scale, pause_value=0):
    n_samples = input_vectors.shape[0]
    nv = input_vectors.shape[1]
    if pause > 0:
        M = np.zeros([2 * n_samples, nv + 1])  # +1 for the time, *2 for the pause
        time_axis_pause = np.arange(0., n_samples * (duration + pause), duration + pause)
        time_axis_data = np.arange(pause, n_samples * (duration + pause), duration + pause)
        time_axis = np.sort(np.concatenate([time_axis_pause, time_axis_data]))
        M[:, 0] = time_axis[:2 * n_samples]
        M[0::2, 1:] = pause_value
        M[1::2, 1:] = input_vectors * scale
        return M
    else:
        M = np.zeros([n_samples, nv + 1])  # +1 for the time, *2 for the pause
        time_axis_data = np.arange(pause, n_samples * (duration + pause), duration + pause)
        time_axis = time_axis_data
        M[:, 0] = time_axis[:n_samples]
        M[0::1, 1:] = input_vectors * scale
        return M


def save_auryn_tiser(filename, tiser, header):
    '''
    Writes a file from a numpy input_vectors (mnist samples and labels)
    '''
    np.savetxt(filename,
               tiser,
               fmt='%f',
               newline='\n',
               delimiter=' ')


#               header = header,
#               comments='')

def save_auryn_wmat(filename, W, dimensions=None, mask=None):
    '''
    Writes a file from a numpy connection matrix *W*, in a format compatible with auryn's 'SparseConnection::load_from_complete_file'.
    mask: boolean matrix specifying which connections should be made. No mask means all connections
    (note that weight 0 creates a connection in auryn)
    '''
    from scipy.sparse import csr_matrix
    from scipy.io import mmwrite

    nv = np.shape(W)[0]
    nh = np.shape(W)[1]

    if np.sum(mask) == 0:
        mmwrite(filename, [[]] * nv, symmetry='general')
        return

    if dimensions is None:
        dimensions = [nv, nh]
    M = np.zeros([nv * nh, 3])

    M[:, 1] = np.arange(nv * nh) % nh
    M[:, 0] = np.repeat(np.arange(nv), nh)
    M[:, 2] = W.flatten()

    if mask is not None:
        M = np.array(filter(lambda x: mask[int(x[0]), int(x[1])], M))

    mmwrite(filename, csr_matrix((M[:, 2], (M[:, 0].astype(int), M[:, 1].astype(int)))), symmetry='general')


def save_all_fwmat(M, prefix):
    from scipy.io import mmwrite
    for k, v in M.iteritems():
        mmwrite(prefix + k, v, symmetry='general')


def to_auryn_wmat(filename, W, dimensions=None, mask=None):
    '''
    Writes a file from a numpy connection matrix *W*, in a format compatible with auryn's 'SparseConnection::load_from_complete_file'.
    mask: boolean matrix specifying which connections should be made. No mask means all connections
    (note that weight 0 creates a connection in auryn)
    '''
    from scipy.sparse import csr_matrix
    from scipy.io import mmwrite

    nv = np.shape(W)[0]
    nh = np.shape(W)[1]

    if np.sum(mask) == 0:
        mmwrite(filename, [[]] * nv, symmetry='general')
        return

    if dimensions is None:
        dimensions = [nv, nh]
    M = np.zeros([nv * nh, 3])

    M[:, 1] = np.arange(nv * nh) % nh
    M[:, 0] = np.repeat(np.arange(nv), nh)
    M[:, 2] = W.flatten()

    if mask is not None:
        M = np.array(filter(lambda x: mask[int(x[0]), int(x[1])], M))

    return csr_matrix((M[:, 2], (M[:, 0], M[:, 1])))


def create_data_rbp(
        output_directory,
        n_samples,
        duration_data,
        duration_pause,
        data_url,
        labels_url,
        nc_perlabel=1,
        filename_current='input_current_file',
        filename_pattern='input_pattern_file',
        beta_prm=1.479,
        min_p=1e-5,
        max_p=.98,
        input_thr=.65,
        input_scale=1.0,
        randomize=False,
        with_labels=False,
        generate_sl=True,
        skip=None,
        limit=None,
        **kwargs
):
    '''
    *data_url* location of training data
    *labels_url* location of labels data
    kwargs passed to load_data_labels
    '''
    print "input_thr is {0}".format(input_thr)
    nv = kwargs["nv"]
    nc = kwargs["nc"]
    n_samples = n_samples
    wake_duration = duration_data
    sleep_duration = duration_pause

    input_vectors, label_vectors, input_labels = load_data_labels(
        data_url=data_url,
        labels_url=labels_url,
        n_samples=n_samples,
        nc_perlabel=nc_perlabel,
        min_p=min_p, max_p=max_p, randomize=randomize,
        skip=skip,
        limit=limit,
        **kwargs)

    if not with_labels:
        label_vectors *= 0
        label_vectors -= 10

    data_vectors = np.concatenate([input_vectors, label_vectors], axis=1)

    tiser = create_tiser(data_vectors,
                         wake_duration,  # Wake
                         sleep_duration,  # Sleep
                         scale=1. / beta_prm,
                         pause_value=0)

    dur = wake_duration / 1e-3

    os.system('mkdir -p inputs/{0}/'.format(output_directory))

    SL = None

    if generate_sl:
        rate = 1. / (4e-3 + 1. / (1e-32 + 5000 * tiser[:, 1:]))
        print "Creating Spike Trains"
        SL = SimSpikingStimulus(rate, time=int(dur), t_sim=len(data_vectors) * dur, with_labels=with_labels)
        print "Exporting evs"
        ev = exportAER(SL, dt=1e-3)
        print "Writing spike trains"
        np.savetxt('inputs/{0}/{1}'.format(output_directory, filename_pattern), ev.get_adtmev(), fmt=('%f', '%d'))
        print "Saved spike trains"
    else:
        header = '#Dataset n_samples:{0} wake:{1} sleep:{2}'.format(n_samples, wake_duration, sleep_duration)
        tiser_data = tiser[:, 1:nv - nc + 1].copy()
        idx = tiser_data > 0
        tiser_data[idx] = (tiser_data[idx] - input_thr) * 10000 * input_scale
        # import pdb; pdb.set_trace()
        # changed '-' to '~'
        tiser_data[~idx] = -10000
        tiser[:, 1:nv - nc + 1] = tiser_data
        tiser[:, (nv - nc + 1):] = (tiser[:, (nv - nc + 1):] - .5) * 10000
        save_auryn_tiser('inputs/{0}/{1}'.format(output_directory, filename_current), tiser, header)
        SL = tiser

        # filename 'inputs/{0}/ecd_modulation_file'.format(directory)

    return input_labels, SL


##### Data Analysis #################

def get_spikelist(filename):
    spikes = np.zeros([0, 2])
    if isinstance(filename, str):
        if filename.find('*') > 0:
            import glob
            filenames = glob.glob(filename)
        else:
            filenames = [filename]
    else:
        assert hasattr(filename, '__len__')
        filenames = filename
    for f in filenames:
        data = pandas_loadtxt_2d(f).T
        if len(data[1, :]) > 1:
            V = data[1, :]
            t = data[0, :] * 1000  # Conversion to [ms]
            spikes = np.concatenate([spikes, zip(V, t)])
    return spikes


def monitor_to_spikelist(filename, id_list=None):
    from pyNCS import pyST
    '''
    Combine Auryn Spike monitor outputs and build a pyNCS SpikeList
    filename can be a string, a string with a wildcard (parsed with glob.glob) or a list of strings, each file must be the output of a spike monitor.
    '''
    spikes = get_spikelist(filename)

    SL = pyST.SpikeList(spikes=spikes, id_list=np.unique(spikes[:, 0]))

    if id_list is not None:
        SL.complete(id_list)

    return SL


def sum_csr(a):
    if len(a) > 0:
        g = a[0].copy()
        for aa in a[1:]:
            g = g + aa
        return g
    else:
        return a


# directory = 'outputs/mnist/train/'
def collect_wmat(directory, con_id):
    from scipy.sparse import csr_matrix
    filenames = '{directory}/coba.*..{0}.*.wmat'.format(con_id, directory=directory)  # Uses wierd file naming by auryn
    from scipy.io import mmread
    a = []
    for f in glob.glob(filenames):
        a.append(mmread(f))

    if numpy_version_largerthan('1.7.0'):
        return csr_matrix(sum(a))
    else:
        if len(a) > 0:
            return csr_matrix(sum_csr(a))
        else:
            # For backward compatibility
            return False


def collect_wmat_auto(directory, con_id):
    from scipy.sparse import csr_matrix
    filenames = '{directory}/coba.*..{0}.*.wmat'.format(con_id, directory=directory)  # Uses wierd file naming by auryn
    from scipy.io import mmread
    a = []
    ggf = glob.glob(filenames)
    if len(ggf) == 0:
        return None, None
    name = extract_wmat_name(ggf[0])
    for f in ggf:
        a.append(mmread(f))

    if numpy_version_largerthan('1.7.0'):
        return csr_matrix(sum(a)), name
    else:
        if len(a) > 0:
            return csr_matrix(sum_csr(a)), name
        else:
            # For backward compatibility
            return False


def extract_wmat_name(filename):
    fh = file(filename, 'r')
    name_cmt_line = [fh.readline() for i in range(3)][-1]
    return name_cmt_line.strip().split('fwmat_')[1].strip('.mtx')


def get_spike_count(directory):
    con_ids = range(5)
    spkcnt = [None for i in range(5)]
    for con_id in con_ids:
        spkcnt[con_id] = collect_gstate(directory, con_id).astype('int')
    return spkcnt


def collect_gstate(directory, con_id):
    filenames = '{directory}/coba.*..{0}.*.gstate'.format(con_id, directory=directory)
    from scipy.io import mmread
    a = np.zeros([0, 2])  # neuron id, bias, spike count
    for f in glob.glob(filenames):
        text = np.loadtxt(f, comments='#', skiprows=3)
        if text.size != 0:
            a = np.concatenate([a, text])
    ia = np.argsort(a[:, 0])
    return a[ia]


def collect_rate(directory, con_id='vmon'):
    filenames = '{directory}/coba.*.{0}.rate'.format(con_id, directory=directory)  # Uses wierd file naming by auryn
    from scipy.io import mmread
    a = np.zeros([0, 3])  # neuron id, bias, spike count
    for f in glob.glob(filenames):
        a = np.concatenate([a, np.loadtxt(f, comments='#', skiprows=3)])
    ia = np.argsort(a[:, 0])
    return a[ia]


def write_parameters_rbp(M, context):
    from scipy.sparse import csr_matrix
    from scipy.io import mmwrite

    for i in ['vh', 'ho']:
        m = M[i]
        mmwrite('{path_to_pkg}/scripts/inputs/{directory}/train/{0}_{1}.mtx'.format('fwmat', i, **context), m,
                symmetry='general')

    if context.has_key('nh1'):
        mmwrite('{path_to_pkg}/scripts/inputs/{directory}/train/{0}_{1}.mtx'.format('fwmat', 'hh', **context), M['hh'],
                symmetry='general')

    return M


def write_allparameters_rbp(M, context):
    from scipy.sparse import csr_matrix
    from scipy.io import mmwrite

    for i in ['vh', 'ho', 've', 'oe', 'eo', 'eh']:
        print 'Writing matrix', i, '{path_to_pkg}/scripts/inputs/{directory}/train/{0}_{1}.mtx'.format('fwmat', i,
                                                                                                       **context)
        m = M[i]
        mmwrite('{path_to_pkg}/scripts/inputs/{directory}/train/{0}_{1}.mtx'.format('fwmat', i, **context), m,
                symmetry='general')

    if context.has_key('nh1'):
        print i, '{path_to_pkg}/scripts/inputs/{directory}/train/{0}_{1}.mtx'.format('fwmat', 'hh', **context)
        mmwrite('{path_to_pkg}/scripts/inputs/{directory}/train/{0}_{1}.mtx'.format('fwmat', 'hh', **context), M['hh'],
                symmetry='general')

    return M


def process_parameters_rbp(context):
    con_id = ['vh',
              'ho',
              've',
              'oe',
              'eo',
              'eh']
    if context.has_key('nh1'):
        con_id += ['hh']
    M = {}
    for i in range(len(con_id)):
        M[con_id[i]] = collect_wmat('{path_to_pkg}/scripts/outputs/{directory}/train/'.format(**context), i)

    write_parameters_rbp(M, context)
    return M


def process_parameters_rbp_dual(context):
    con_id = ['vh',
              'ho',
              've',
              've2',
              'oe',
              'oe2',
              'eo',
              'eo2',
              'eh',
              'eh2']
    if context.has_key('nh1'):
        con_id += ['hh']
    M = {}
    for i in range(len(con_id)):
        M[con_id[i]] = collect_wmat('{path_to_pkg}/scripts/outputs/{directory}/train/'.format(**context), i)

    write_parameters_rbp(M, context)
    return M


def process_parameters_auto(context):
    M = {}
    i = 0
    while (True):
        m, name = collect_wmat_auto('{path_to_pkg}/scripts/outputs/{directory}/train/'.format(**context), i)
        if name == None:
            break
        else:
            M[name] = m
            i += 1
    save_all_fwmat(M, '{path_to_pkg}/scripts/inputs/{directory}/train/fwmat_'.format(**context))
    return M


def process_allparameters_rbp(context):
    con_id = ['vh',
              'ho',
              've',
              'oe',
              'eo',
              'eh']
    if context.has_key('nh1'):
        con_id += ['hh']
    M = {}
    for i in range(len(con_id)):
        M[con_id[i]] = collect_wmat('{path_to_pkg}/scripts/outputs/{directory}/train/'.format(**context), i)

    write_allparameters_rbp(M, context)
    return M


def read_allparamters_dual(context):
    con_id = ['vh',
              'ho',
              've',
              've2',
              'oe',
              'oe2',
              'eo',
              'eo2',
              'eh',
              'eh2']
    if context.has_key('nh1'):
        con_id += ['hh']
    M = {}
    for i in range(len(con_id)):
        M[con_id[i]] = collect_wmat('{path_to_pkg}/scripts/outputs/{directory}/train/'.format(**context), i)

    return M


def process_test_rbp(context):
    Sc = monitor_to_spikelist('outputs/{directory}/test/coba.*.out.ras'.format(**context))
    Sc.complete(range(context['nc']))
    Sc.t_start = 0
    Sc.t_stop = context['simtime_test'] * 1000
    T = (context['sample_duration_test'] + context['sample_pause_test']) * 1000
    fr = Sc.firing_rate(T).reshape(context['nc'], context['nc_perlabel'], -1).mean(axis=1)
    return np.argmax(fr, axis=0)


def get_confusion_df(pred_labels, actual_labels, normalized=False, margins=False):
    y_actu = pd.Series(actual_labels, name='Actual label')
    y_pred = pd.Series(pred_labels, name='Predicted')
    df_confusion = pd.crosstab(y_actu, y_pred, margins=margins).transpose()
    if normalized:
        df_confusion = df_confusion / df_confusion.sum(axis=0)
    return df_confusion


def process_test_classification(context, sample_duration_test, actual_labels):
    raw_data = get_spikelist('{path_to_pkg}/scripts/outputs/{directory}/test/coba.*.out.ras'.format(**context))
    raw_data[:, 1] = raw_data[:, 1] / 1000
    split_at = raw_data[:, 1].searchsorted(sample_duration_test)
    split_raw = np.split(raw_data, split_at[:-1])

    pred_rate_labels = get_rate_prediction(split_raw)
    pred_first_labels = get_first_prediction(split_raw)

    rate_class = classify(pred_rate_labels, actual_labels)
    first_class = classify(pred_first_labels, actual_labels)

    rate_confusion_data_frame = get_confusion_df(pred_rate_labels, actual_labels, normalized=True)
    first_confusion_data_frame = get_confusion_df(pred_first_labels, actual_labels, normalized=True)

    if context['directory'] == 'dvs_gesture_split':
        label_start = True
    else:
        label_start = False
    output_spikes_per_label = get_output_spikes_per_label(split_raw, actual_labels, context['nc'], label_start)
    ouput_spikes_per_label_norm = get_output_spikes_per_label(split_raw, actual_labels, context['nc'], label_start,
                                                              normalized=True)

    total_snr = calc_total_snr(split_raw, label_start, actual_labels, context['nc'])
    snr_per_label = calc_snr_per_label(output_spikes_per_label.T)

    # print('rate_classification: {}'.format(rate_class))
    # print('first_classification: {}'.format(first_class))
    return rate_class, first_class, rate_confusion_data_frame, first_confusion_data_frame, output_spikes_per_label, ouput_spikes_per_label_norm, total_snr, snr_per_label


def process_prediction(context):
    raw_data = get_spikelist('{path_to_pkg}/scripts/outputs/{directory}/predict/coba.*.out.ras'.format(**context))
    output_spikes = [i[0] for i in raw_data]
    label_bins = np.bincount(output_spikes, minlength=context['nc'])
    return np.argmax(label_bins)


def get_output_spikes_per_label(split_raw, actual_labels, nc, start_from_one, normalized=False):
    s_z = 0
    if start_from_one:
        s_z = 1
    label_bins = np.zeros((nc - s_z, nc - s_z))
    for i, elem in enumerate(split_raw):
        label_bins[actual_labels[i] - s_z] += np.bincount(elem[:, 0].astype(int), minlength=nc)[s_z:]
    actual_label_bins = np.bincount(actual_labels)[s_z:]
    for i, bin in enumerate(actual_label_bins):
        label_bins[i] /= bin
    if normalized:
        label_bins = map(lambda x: x / np.sum(x), label_bins)
    return np.array(label_bins).T


def calc_total_snr(split_raw, start_from_one, actual_labels, nc):
    overall_wrong_spikes = 0
    overall_correct_spikes = 0
    s_z = 0
    if start_from_one:
        s_z = 1
    for i, elem in enumerate(split_raw):
        label_bins = np.bincount(elem[:, 0].astype(int), minlength=nc)[s_z:]
        overall_wrong_spikes += np.sum(label_bins[:actual_labels[i] - s_z]) + np.sum(
            label_bins[actual_labels[i] + 1 - s_z:])
        overall_correct_spikes += label_bins[actual_labels[i] - s_z]
    if overall_correct_spikes == 0:
        total_snr = 0.
    elif overall_wrong_spikes == 0:
        total_snr = np.inf
    else:
        total_snr = float(overall_correct_spikes) / overall_wrong_spikes
    return total_snr


def calc_snr_per_label(output_spikes):
    snr_per_label = []
    for i, label in enumerate(output_spikes):
        sum_of_wrong_spikes = np.sum(label[:i]) + np.sum(label[i + 1:])
        number_of_correct_spikes = label[i]
        if sum_of_wrong_spikes > 0:
            snr = number_of_correct_spikes / sum_of_wrong_spikes
        else:
            if number_of_correct_spikes > 0:
                snr = np.inf
            else:
                snr = 0.
        snr_per_label.append(snr)
    return snr_per_label


def get_rate_prediction(split_raw):
    predicted_labels = []
    for elem in split_raw:
        bins = np.bincount(elem[:, 0].astype(int))
        if bins.size > 0:
            predicted_labels.append(np.argmax(bins))
        else:
            predicted_labels.append(-1)
    return predicted_labels


def get_first_prediction(split_raw):
    prediction_labels = []
    for elem in split_raw:
        if elem.size > 0:
            prediction_labels.append(int(elem[:, 0][0]))
    return prediction_labels


def classify(pred_labels, labels_test):
    return float(len(filter(lambda x: x[0] == x[1], zip(pred_labels, labels_test)))) / len(labels_test)


def process_rasters(directory, N, t_stop, t_sample):
    Sh = monitor_to_spikelist(directory)
    Sh.complete(range(0, N))
    Sh.t_start = 0
    Sh.t_stop = t_stop * 1000
    fr = Sh.firing_rate(t_sample * 1000)
    return fr


def plot_epochs(n_epochs, wstats_mean, wstats_std, acc_hist):
    import pylab
    ah = np.array(acc_hist)
    color = ['b', 'g', 'r', 'k']
    pylab.ion()
    pylab.figure()
    ax1 = pylab.axes()
    pylab.figure()
    ax2 = pylab.axes()
    ax2.plot(ah[:, 0], ah[:, 1])

    for i in range(n_epochs):
        ax1.clear()
        for j in range(4):
            ax1.plot(range(n_epochs), wstats_mean[j, :], color=color[j], label='mean vh')
            ax1.fill_between(range(n_epochs), wstats_mean[j, :] + wstats_std[j, :],
                             wstats_mean[j, :] - wstats_std[j, :], color=color[j], alpha=.5)


def plot_recognition_progress(labels_test, context):
    Sc = monitor_to_spikelist('outputs/{directory}/test/coba.*.c.ras'.format(**context))
    Sc.complete(range(context['nv'] - context['nc'], context['nv']))
    import pylab
    res = []
    tbin = 10  # must be int
    time_axis = np.arange(0., tbin, context['sample_duration_test']) * 1000
    s = [None] * context['n_samples_test']
    for i in range(context['n_samples_test']):
        print i
        s[i] = Sc.time_slice(i * context['sample_duration_test'] * 1000,
                             i * context['sample_duration_test'] * 1000 + context[
                                 'sample_duration_test'] * 1000).firing_rate(tbin).cumsum(axis=1)
        s[i] /= np.arange(1, s[i].shape[1] + 1)
        res = np.argmax(s, axis=1)


#        res.append(float(sum(process_test_deltat(t, context) == labels_test))/len(labels_test))
#    pylabte.plot(time_axis, res)
#    pylab.axhline(np.mean(res[len(res)/2:]))
#    return time_axis,res

def scale_to_unit_interval(ndar, eps=1e-8):
    """ Scales all values in the ndarray ndar to be between 0 and 1 """
    ndar = ndar.copy()
    ndar -= ndar.min()
    ndar *= 1.0 / (ndar.max() + eps)
    return ndar


def tile_raster_images(X, img_shape, tile_shape, tile_spacing=(0, 0),
                       scale_rows_to_unit_interval=False,
                       output_pixel_vals=False):
    """
    Transform an array with one flattened image per row, into an array in
    which images are reshaped and layed out like tiles on a floor.

    This function is useful for visualizing datasets whose rows are images,
    and also columns of matrices for transforming those rows
    (such as the first layer of a neural net).

    :type X: a 2-D ndarray or a tuple of 4 channels, elements of which can
    be 2-D ndarrays or None;
    :param X: a 2-D array in which every row is a flattened image.

    :type img_shape: tuple; (height, width)
    :param img_shape: the original shape of each image

    :type tile_shape: tuple; (rows, cols)
    :param tile_shape: the number of images to tile (rows, cols)

    :param output_pixel_vals: if output should be pixel values (i.e. int8
    values) or floats

    :param scale_rows_to_unit_interval: if the values need to be scaled before
    being plotted to [0,1] or not


    :returns: array suitable for viewing as an image.
    (See:`PIL.Image.fromarray`.)
    :rtype: a 2-d array with same dtype as X.

    """

    assert len(img_shape) == 2
    assert len(tile_shape) == 2
    assert len(tile_spacing) == 2

    # The expression below can be re-written in a more C style as
    # follows :
    #
    # out_shape    = [0,0]
    # out_shape[0] = (img_shape[0]+tile_spacing[0])*tile_shape[0] -
    #                tile_spacing[0]
    # out_shape[1] = (img_shape[1]+tile_spacing[1])*tile_shape[1] -
    #                tile_spacing[1]
    out_shape = [(ishp + tsp) * tshp - tsp for ishp, tshp, tsp
                 in zip(img_shape, tile_shape, tile_spacing)]

    if isinstance(X, tuple):
        assert len(X) == 4
        # Create an output numpy ndarray to store the image
        if output_pixel_vals:
            out_array = np.zeros((out_shape[0], out_shape[1], 4),
                                 dtype='uint8')
        else:
            out_array = np.zeros((out_shape[0], out_shape[1], 4),
                                 dtype=X.dtype)

        # colors default to 0, alpha defaults to 1 (opaque)
        if output_pixel_vals:
            channel_defaults = [0, 0, 0, 255]
        else:
            channel_defaults = [0., 0., 0., 1.]

        for i in xrange(4):
            if X[i] is None:
                # if channel is None, fill it with zeros of the correct
                # dtype
                dt = out_array.dtype
                if output_pixel_vals:
                    dt = 'uint8'
                out_array[:, :, i] = np.zeros(out_shape,
                                              dtype=dt) + channel_defaults[i]
            else:
                # use a recurrent call to compute the channel and store it
                # in the output
                out_array[:, :, i] = tile_raster_images(
                    X[i], img_shape, tile_shape, tile_spacing,
                    scale_rows_to_unit_interval, output_pixel_vals)
        return out_array

    else:
        # if we are dealing with only one channel
        H, W = img_shape
        Hs, Ws = tile_spacing

        # generate a matrix to store the output
        dt = X.dtype
        if output_pixel_vals:
            dt = 'uint8'
        out_array = np.zeros(out_shape, dtype=dt)

        for tile_row in xrange(tile_shape[0]):
            for tile_col in xrange(tile_shape[1]):
                if tile_row * tile_shape[1] + tile_col < X.shape[0]:
                    this_x = X[tile_row * tile_shape[1] + tile_col]
                    if scale_rows_to_unit_interval:
                        # if we should scale values to be between 0 and 1
                        # do this by calling the `scale_to_unit_interval`
                        # function
                        this_img = scale_to_unit_interval(
                            this_x.reshape(img_shape))
                    else:
                        this_img = this_x.reshape(img_shape)
                    # add the slice to the corresponding position in the
                    # output array
                    c = 1
                    if output_pixel_vals:
                        c = 255
                    out_array[
                    tile_row * (H + Hs): tile_row * (H + Hs) + H,
                    tile_col * (W + Ws): tile_col * (W + Ws) + W
                    ] = this_img * c
        return out_array


# def plot_rasters(context, directory, t_start=0, t_stop=1000, kwargs={'marker': '.', 'markersize': 1, 'color': 'k'},
#                  t_shift=0):
#     import pyNCS.pyST
#     from matplotlib.ticker import MaxNLocator
#     matplotlib.rcParams['figure.subplot.top'] = .95
#     matplotlib.rcParams['figure.subplot.bottom'] = .2
#     matplotlib.rcParams['figure.subplot.left'] = .25
#     matplotlib.rcParams['figure.subplot.right'] = .95
#     matplotlib.rcParams['figure.subplot.hspace'] = .4
#     pylab.figure(figsize=(5, 5))
#     nv = context['nv']
#     nh = context['nh']
#     nc = context['nc']
#     Sh = monitor_to_spikelist(directory + '/coba.*.h.ras').time_slice(t_start, t_stop)
#     Sh.time_offset(t_shift)
#     Sh.complete(range(nh))
#     Sv = monitor_to_spikelist(directory + '/coba.*.v.ras').time_slice(t_start, t_stop)
#     Sv.time_offset(t_shift)
#     Sv.complete(range(nv - nc))
#     Sc = monitor_to_spikelist(directory + '/coba.*.c.ras').time_slice(t_start, t_stop)
#     Sc.time_offset(t_shift)
#     Sc.complete(range(nv - nc, nv))
#     Sc_new = pyNCS.pyST.STsl.mapSpikeListAddresses(Sc)
#
#     axv = pylab.subplot(311)
#     Sv.raster_plot(kwargs=kwargs, display=axv)
#     pylab.xticks([])
#     pylab.xlabel('')
#     pylab.yticks([0, (nv - nc) / 2, (nv - nc)])
#     pylab.ylabel('Data')
#     axh = pylab.subplot(312)
#     Sh.raster_plot(kwargs=kwargs, display=axh)
#     pylab.ylabel('Hidden')
#     pylab.xticks([])
#     pylab.xlabel('')
#     pylab.yticks([0, nh / 2, nh])
#     axc = pylab.subplot(313)
#     Sc_new.raster_plot(kwargs=kwargs, display=axc)
#     pylab.ylabel('Class')
#     pylab.yticks([0, nc / 2, nc])
#     pylab.ylim([-1, nc])
#     axv.yaxis.set_label_coords(-.2, 0.5)
#     axh.yaxis.set_label_coords(-.2, 0.5)
#     axc.yaxis.set_label_coords(-.2, 0.5)
#     axc.xaxis.set_major_locator(MaxNLocator(4))
#     return Sv, Sh, Sc


def exportAER(spikeLists,
              format='t',
              isi=False,
              dt=1,
              debug=False,
              *args, **kwargs):
    '''
    Modified from pyNCS.pyST.exportAER
    '''
    from pyNCS import pyST

    out = []
    assert format in ['t', 'a'], 'Format must be "a" or "t"'
    ev = pyST.events(atype='logical')

    # Translate logical addresses to physical using a mapping
    if isinstance(spikeLists, pyST.SpikeList):
        slrd = spikeLists.raw_data()
        if len(slrd) > 0:
            tmp_mapped_SL = np.fliplr(slrd)
            mapped_SL = np.zeros_like(tmp_mapped_SL, dtype='float')
            mapped_SL[:, 1] = tmp_mapped_SL[:, 1] * dt  # ms
            mapped_SL[:, 0] = tmp_mapped_SL[:, 0]
            ev.add_adtmev(mapped_SL)
            # ev.add_adtmev(mapSpikeListAddresses(spikeLists[ch],mappin
            # g).convert(format='[id,time*1000]'))

    if debug:
        print("Address encoding took {0} seconds".format(tictoc))

    # Multiplex
    sortedIdx = np.argsort(ev.get_tm())

    # Create new sorted events object
    if len(sortedIdx) > 0:
        ev = pyST.events(ev.get_tmadev()[sortedIdx, :], atype='l')
        # exportAER
        if isi:
            ev.set_isi()

    else:
        ev = pyST.events(atype='l')

    # Choose desired output: no filename given, return events
    return ev


def SimSpikingStimulus(stim, time=1000, t_sim=None, with_labels=True, nc=10):
    '''
    Times must be sorted. ex: times = [0, 1, 2] ; scale = [1,0]
    *poisson*: integer, output is a poisson process with mean
    data/poisson, scaled by *poisson*.
    '''
    from pyNCS import pyST
    n = np.shape(stim)[1]
    SL = pyST.SpikeList(id_list=range(n))
    SLd = pyST.SpikeList(id_list=range(n - nc))
    SLc = pyST.SpikeList(id_list=range(n - nc, n))
    for i in range(n - nc):
        SLd[i] = pyST.STCreate.inh_poisson_generator(stim[:, i],
                                                     range(0, len(stim) * time, time),
                                                     t_stop=t_sim)
    if with_labels:
        for t in range(0, len(stim)):
            SLt = pyST.SpikeList(id_list=range(n - nc, n))
            for i in range(n - nc, n):
                if stim[t, i] > 1e-2:
                    SLt[i] = pyST.STCreate.regular_generator(stim[t, i],
                                                             jitter=True,
                                                             t_start=t * time,
                                                             t_stop=(t + 1) * time)
            if len(SLt.raw_data()) > 0: SLc = pyST.merge_spikelists(SLc, SLt)

    if len(SLc.raw_data()) > 0:
        SL = pyST.merge_spikelists(SLd, SLc)
    else:
        SL = SLd
    return SL
