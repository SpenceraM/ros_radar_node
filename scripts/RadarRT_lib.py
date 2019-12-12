import numpy as np
import struct  # used with binary data


def cfg_to_dict(file_path):
    # make sure the order of the headers and functions match
    cfg_headers = ['channelCfg','profileCfg','chirpCfg','frameCfg','adcCfg','lowPower']
    cfg_funcs = [channelStr_to_dict, profileStr_to_dict, chirp_to_dict,frameStr_to_dict,adcStr_to_dict,power_to_dict]
    cfg = {}
    with open(file_path, 'r',) as file:
        for line in file:
            for header,func in zip(cfg_headers,cfg_funcs):
                if header in line:
                    line = line.split('\'')[1::2]  # Gets info between quotes
                    line = line[0].split()  # splits into list
                    cfg = func(line[1:], cfg)
    return cfg


def channelStr_to_dict(args, curr_cfg=None):

    if curr_cfg:
        cfg = curr_cfg
    else:
        cfg = {}

    # This is the number of receivers which is equivalent to the number of lanes in the source code
    # Later, may include the result from the number of transmitters
    rx_bin = bin(int(args[0]))[2:].zfill(4)
    cfg['numLanes'] = len([ones for ones in rx_bin if ones == '1'])
    (cfg['rx4'],cfg['rx3'],cfg['rx2'],cfg['rx1']) = [bool(int(ones)) for ones in rx_bin]

    # This is the number of transmitters
    tx_bin = bin(int(args[1]))[2:].zfill(3)
    cfg['numTx'] = len([ones for ones in tx_bin if ones == '1'])
    (cfg['tx3'], cfg['tx2'], cfg['tx1']) = [bool(int(ones)) for ones in tx_bin]
    print('[NOTE] Azimuth angle can be determined from channel config.') if cfg['tx2'] is True and (cfg['tx1'] or cfg['tx3']) is False else 0
    print('[NOTE] Azimuth angle can be determined from channel config.') if cfg['tx2'] is False and (cfg['tx1'] or cfg['tx3']) is True else 0
    print('[NOTE] Elevation and Azimuth angle can be determined from channel config.') if cfg['tx2'] is True and (cfg['tx1'] or cfg['tx3']) else 0


    return cfg


def profileStr_to_dict(args, curr_cfg=None):
    normalizer = [None, 1e9, 1e-6, 1e-6, 1e-6, None, None, 1e12, 1e-6, None, 1e3, None, None, None]
    dtype = [int, float, float, float, float, float, float, float, float, int, float, int, int, float]
    keys = ['id',
            'start_frequency',
            'idle',
            'adcStartTime',
            'rampEndTime',
            'txPower',
            'txPhaseShift',
            'freqSlopeConst',
            'txStartTime',
            'adcSamples',
            'adcSampleRate',
            'hpfCornerFreq1',
            'hpfCornerFreq2',
            'rxGain',
            ]
    # Check if the main dictionary exists
    if curr_cfg:
        cfg = curr_cfg
        if 'profiles' not in cfg.keys():
            cfg['profiles']=[]
    else:
        cfg = {'profiles': []}

    profile_dict = {}
    for k, v, n, d in zip(keys, args, normalizer, dtype):
        profile_dict[k] = d(float(v) * n if n else v)

    cfg['profiles'].append(profile_dict)
    return cfg


def chirp_to_dict(args,curr_cfg=None):
    if curr_cfg:
        cfg = curr_cfg
        if 'chirps' not in cfg.keys():
            cfg['chirps'] = []
    else:
        cfg = {'chirps': []}

    chirp_dict = {}
    chirp_dict['chirpStartIndex'] = int(args[0])
    chirp_dict['chirpStopIndex'] = int(args[1])
    chirp_dict['profileID'] = int(args[2])
    chirp_dict['startFreqVariation'] = float(args[3])
    chirp_dict['slopeVariation'] = float(args[4])
    chirp_dict['idleVariation'] = float(args[5])
    chirp_dict['adcStartVariation'] = float(args[6])

    tx_bin = bin(int(args[7]))[2:].zfill(3)
    (chirp_dict['chirptx3'], chirp_dict['chirptx2'], chirp_dict['chirptx1']) = [bool(int(ones)) for ones in tx_bin]

    cfg['chirps'].append(chirp_dict)
    return cfg


def power_to_dict(args,curr_cfg=None):
    if curr_cfg:
        cfg = curr_cfg
    else:
        cfg = {}
    if int(args[1]) ==1:
        cfg['adcPower'] = 'low'
        print('[NOTE] The Low power ADC mode limits the sampling rate to half the max value.')
    elif int(args[1]) ==0:
        cfg['adcPower'] = 'regular'
    else:
        raise ValueError ("Invalid Power Level")
    return cfg


def frameStr_to_dict(args, cfg):

    # Number of chirps
    if 'chirps' not in cfg.keys():
        raise ValueError("Need to define chirps before frame")

    chirpStop =0
    for ii in range(len(cfg['chirps'])):
        chirpStop = max(chirpStop,cfg['chirps'][ii]['chirpStopIndex'])
    chirps_len = chirpStop + 1

    cfg['numChirps'] = int(args[2]) * chirps_len  # num loops * len(chirps)

    # args[4] is the time in milliseconds of each frame
    cfg['fps'] = 1000/float(args[4])

    if int(args[6]) != 0: cfg['numFrames'] = int(args[6])

    return cfg


def adcStr_to_dict(args, curr_cfg=None):
    if curr_cfg:
        cfg = curr_cfg
    else:
        cfg = {}

    if int(args[1]) == 1:
        cfg['isComplex'] = True
        cfg['image_band'] = False
        print('[NOTE] Complex 1x mode, Only Real IF Spectrum is filtered and sent to ADC, so if Sampling rate\n'
              '       is X, ADC data would include frequency spectrum from 0 to X.')
    elif int(args[1]) == 2:
        cfg['isComplex'] = True
        cfg['image_band'] = True
        print('[NOTE] Complex 2x mode, both Imaginary and Real IF spectrum is filtered and sent to ADC, so\n'
              '       if Sampling rate is X, ADC data would include frequency spectrum from -X/2 to X/2.')
    else:
        raise ValueError("Real Data Type Not Supported")

    return cfg


def dict_to_cli(cfg,file_path):

    f = open(file_path,'w+')
    f.write('iwr_cfg_cmd = [ \\\n')  # mandatory
    f.write('        \'flushCfg\', \\\n')  # mandatory
    f.write('        \'dfeDataOutputMode 1\', \\\n')  # frame based chirps

    # rx antennas/lanes for channel config
    rx_bool = [cfg['rx4'], cfg['rx3'], cfg['rx2'], cfg['rx1']]
    rx_mask = sum(2 ** i for i, v in enumerate(reversed(rx_bool)) if v)
    # number of tx antennas for channel config
    tx_bool = [cfg['tx3'], cfg['tx2'], cfg['tx1']]
    tx_mask = sum(2 ** i for i, v in enumerate(reversed(tx_bool)) if v)
    print('[NOTE] Azimuth angle can be determined from channel config.') if cfg['tx2'] is True and (cfg['tx1'] or cfg['tx3']) is False else 0
    print('[NOTE] Azimuth angle can be determined from channel config.') if cfg['tx2'] is False and (cfg['tx1'] or cfg['tx3']) is True else 0
    print('[NOTE] Elevation and Azimuth angle can be determined from channel config.') if cfg['tx2'] is True and (cfg['tx1'] or cfg['tx3']) else 0
    f.write('        \'channelCfg %s %s 0\', \\\n' % (rx_mask, tx_mask))  # rx and tx mask

    # adc config
    if cfg['isComplex'] and cfg['image_band']:
        outputFmt = 2
        print('[NOTE] Complex 2x mode, both Imaginary and Real IF spectrum is filtered and sent to ADC, so\n'
              '       if Sampling rate is X, ADC data would include frequency spectrum from -X/2 to X/2.')
    elif cfg['isComplex'] and not cfg['image_band'] == True:
        outputFmt = 1
        print('[NOTE] Complex 1x mode, Only Real IF Spectrum is filtered and sent to ADC, so if Sampling rate\n'
              '       is X, ADC data would include frequency spectrum from 0 to X.')
    else: raise ValueError("Real Data Type Not Supported")
    f.write('        \'adcCfg 2 %s\', \\\n' % outputFmt)  # 16 bits (mandatory), complex 1x or 2x

    # adc power
    if cfg['adcPower'] =='low':
        power_mode = 1
        print('[NOTE] The Low power ADC mode limits the sampling rate to half the max value.')
    elif cfg['adcPower'] =='regular': power_mode = 0
    else: raise ValueError("ADC power level Not Supported")
    f.write('        \'lowPower 0 %s\', \\\n' % power_mode)  # power mode

    # profile configs
    for profile_ii in cfg['profiles']:
        f.write('        \'profileCfg %s %s %s %s %s %s %s %s %s %s %s %s %s %s\', \\\n'
                % (profile_ii['id'],
                float(profile_ii['start_frequency']/1e9),
                float(profile_ii['idle']/1e-6),
                float(profile_ii['adcStartTime']/1e-6),
                float(profile_ii['rampEndTime']/1e-6),
                float(profile_ii['txPower']),
                float(profile_ii['txPhaseShift']),
                float(profile_ii['freqSlopeConst']/1e12),
                float(profile_ii['txStartTime']/1e-6),
                int(profile_ii['adcSamples']),
                float(profile_ii['adcSampleRate']/1e3),
                int(profile_ii['hpfCornerFreq1']),
                int(profile_ii['hpfCornerFreq2']),
                float(profile_ii['rxGain'])))

    # chirp configs
    for chirp_ii in cfg['chirps']:

        # Check if chirp is referring to valid profile config
        profile_valid = False
        for profile_ii in cfg['profiles']:
            if chirp_ii['profileID'] == profile_ii['id']: profile_valid = True
        if profile_valid is False: raise ValueError("The following profile id used in chirp "
                                                    "is invalid: %i" % chirp_ii['profileID'])
        ###############################################################################################################
        # check if tx values are valid
        # if hamming([chirp_ii['chirptx3'],chirp_ii['chirptx2'],chirp_ii['chirptx1']],
        #     [cfg['tx3'], cfg['tx2'], cfg['tx1']])*3 > 1:
        #     raise ValueError("Chirp should have at most one different Tx than channel cfg")
        ###############################################################################################################
        if chirp_ii['chirpStartIndex'] > chirp_ii['chirpStopIndex']: raise ValueError("Particular chirp start index after chirp stop index")
        tx_bool = [chirp_ii['chirptx3'],chirp_ii['chirptx2'],chirp_ii['chirptx1']]
        tx_mask = sum(2 ** i for i, v in enumerate(reversed(tx_bool)) if v)
        f.write('        \'chirpCfg %s %s %s %s %s %s %s %s\', \\\n'
                % (chirp_ii['chirpStartIndex'],
                   chirp_ii['chirpStopIndex'],
                   chirp_ii['profileID'],
                   chirp_ii['startFreqVariation'],
                   chirp_ii['slopeVariation'],
                   chirp_ii['idleVariation'],
                   chirp_ii['adcStartVariation'],
                   tx_mask))

    # frame config
    chirpStop = 0
    chirpStart = 511  # max value for chirp start index
    for chirp_ii in cfg['chirps']:
        chirpStop = max(chirpStop, chirp_ii['chirpStopIndex'])
        chirpStart = min(chirpStart,chirp_ii['chirpStartIndex'])
    chirps_len  = chirpStop + 1

    numLoops = cfg['numChirps']/chirps_len
    if chirpStart > chirpStop: raise ValueError("Chirp(s) start index is after chirp stop index")
    if numLoops % 1 != 0: raise ValueError("Number of loops is not integer")
    if numLoops > 255 or numLoops < 1: raise ValueError("Number of loops must be int in [1,255]")

    numFrames = cfg['numFrames'] if 'numFrames' in cfg.keys() else 0  # if zero => inf

    f.write('        \'frameCfg %s %s %s 0 %s 1 %s\', \\\n'
            % (chirpStart, chirpStop, int(numLoops), int(1000/cfg['fps']), numFrames))

    f.write('        \'testFmkCfg 0 0 0 1\', \\\n')
    f.write('        \'setProfileCfg disable ADC disable\'\n    ]')
    f.close()

##########################################
# Helper functions to get physical units #
##########################################


def compute_interchirp_time(cfg, profile = 0):
    return cfg['profiles'][profile]['idle']+cfg['profiles'][profile]['rampEndTime']


def fft_bin2freq(fft_bin, cfg, fft_shift=False, profile = 0):
    #TODO: handle fftshift?
    return np.fft.fftfreq(cfg['profiles'][profile]['adcSamples'], 1/cfg['profiles'][profile]['adcSampleRate'])[fft_bin]


def freq2range(beat_freq, cfg, profile = 0):
    c = 299792458.0
    return beat_freq * c/(2*cfg['profiles'][profile]['freqSlopeConst'])


def fft_bin2range(fft_bin, cfg):
    return freq2range(fft_bin2freq(fft_bin, cfg), cfg)


def doppler_bin2velocity(doppler_bin, cfg, shifted=True, profile = 0):
    c = 299792458.0
    tc = compute_interchirp_time(cfg)
    lambd = c/cfg['profiles'][profile]['start_frequency']
    dv = lambd/(2*cfg['numChirps']*tc)
    v = np.arange(cfg['numChirps'])*dv
    if shifted:
        zero_idx = np.argmin(np.fft.fftshift(np.arange(cfg['numChirps'])))
        v -= v[zero_idx]
    return v[doppler_bin]

#######################
# IO Helper functions #
#######################


def read_raw_udp(filename, PACKET_LENGTH=1466):
    with open(filename, 'rb') as raw_capture:
        packet_data = []
        for chunk in iter(lambda: raw_capture.read(PACKET_LENGTH), ''):
            if len(chunk) < PACKET_LENGTH:
                print('Remainder bytes: {}. Something is wrong if not 0'.format(len(chunk)))
                break
            else:
                packet_data.append(chunk)
        print('{} packets read.'.format(len(packet_data)))

    return packet_data


def zeropad_missing_packets(packet_data, config, verbose=True, profile = 0):
    # TODO: implement reorder
    seqn_ = 0
    bytec_ = 0
    samples = []

    numBytesPerFrame = 2 * config['profiles'][profile]['adcSamples'] * config['numLanes'] * config['numChirps']
    if config['isComplex']:
        numBytesPerFrame *= 2
    numSamplesPerFrame = numBytesPerFrame / 2
    print("Bytes per frame: {}".format(numBytesPerFrame))
    packetsPerFrame = numBytesPerFrame / 1456
    print("Packets per frame: {}".format(packetsPerFrame))

    for packet in packet_data:
        seqn, bytec = struct.unpack('<IIxx', packet[:10])
        print(seqn)
        if seqn_ + 1 != seqn:
            packets_lost = seqn - seqn_
            bytes_lost = bytec - bytec_
            if verbose:
                print("@frame {}-{}: lost {} packets ({} bytes, {} samples, {:.3f} frames)".format(
                    seqn_ / packetsPerFrame,
                    seqn / packetsPerFrame,
                    packets_lost,
                    bytes_lost,
                    (bytes_lost) / 2,
                    bytes_lost / numBytesPerFrame
                    ))

            samples.append(np.zeros((bytec - bytec_ - 1456) // 2, dtype=np.int16))
        if seqn == 11*360-1:
            print()
        samples.append(np.frombuffer(packet[10:], dtype=np.int16))
        seqn_ = seqn
        bytec_ = bytec
    samples = np.concatenate(samples)
    print("Last seqn read {}, last bytecount {} + 1456".format(seqn, bytec))
    print("expected samples length {}".format((bytec + 1456) // 2))
    print("Num samples {}".format(samples.shape))

    print("{} frames available:".format(samples.shape[0] / numSamplesPerFrame))

    return samples


def form_data_cube(config, flat_array, profile =0):
    samples_per_frame = config['profiles'][profile]['adcSamples'] * config['numChirps'] * config['numLanes']
    if config['isComplex']:
        data = flat_array.reshape(-1, 8)
        data = data[:, :4] + 1j * data[:, 4:]
        data.reshape(-1)
    else:
        data = flat_array

    samples_to_discard = len(data) % samples_per_frame

    if samples_to_discard > 0:
        print("Warning: there seems to be incomplete frames. {} samples discarded.".format(samples_to_discard))
        data = data[:-samples_to_discard]

    return data.reshape(-1, config['numChirps'], config['profiles'][profile]['adcSamples'], config['numLanes'])
