## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):
    module = bld.create_ns3_module('uan', ['network', 'energy', 'mobility'])
    module.source = [
        'model/uan-channel.cc',
        'model/uan-phy-gen.cc',
        'model/uan-phy-real.cc',##
        'model/uan-phy-real-newb.cc',##
        'model/uan-phy-real-new.cc',##
        'model/uan-phy-real-cloud.cc',##
        'model/uan-phy-mfsk.cc',
        'model/uan-mac.cc',
        'model/uan-transducer.cc',
        'model/uan-transducer-hd.cc',
        'model/uan-net-device.cc',
        'model/uan-tx-mode.cc',
        'model/uan-prop-model.cc',
        'model/uan-prop-model-ideal.cc',
        'model/uan-mac-aloha.cc',
        ##'model/uan-mac-pure-aloha.cc',##
        'model/uan-mac-new-aloha.cc',##
        'model/uan-mac-maca.cc',##
        'model/uan-mac-fc-maca.cc',##
        'model/uan-emulation-packet.cc',##
        'model/uan-emulation-manager.cc',##
        'model/uan_tool.cc',##
        'model/uan-header-common.cc',
        'model/uan-header-new.cc',##
        'model/uan-header-fc.cc',##
        'model/uan-noise-model-default.cc',
        'model/uan-mac-cw.cc',
        'model/uan-prop-model-thorp.cc',
        'model/uan-phy-dual.cc',
        'model/uan-header-rc.cc',
        'model/uan-mac-rc.cc',
        'model/uan-mac-rc-gw.cc',
        'model/uan-phy.cc',
        'model/uan-noise-model.cc',
        'model/acoustic-modem-energy-model.cc',
        'helper/uan-helper.cc',
        'helper/acoustic-modem-energy-model-helper.cc',
        ]

    module_test = bld.create_ns3_module_test_library('uan')
    module_test.source = [
        'test/uan-test.cc',
        'test/uan-energy-model-test.cc',
        ]
    headers = bld(features='ns3header')
    headers.module = 'uan'
    headers.source = [
        'model/uan-channel.h',
        'model/uan-phy.h',
        'model/uan-mac.h',
        'model/uan-net-device.h',
        'model/uan-prop-model.h',
        'model/uan-tx-mode.h',
        'model/uan-transducer.h',
        'model/uan-phy-gen.h',
        'model/uan-phy-real.h',##
        'model/uan-phy-real-newb.h',##multiple thread &&different length pkt
        'model/uan-phy-real-new.h',##multiple thread
        'model/uan-phy-real-cloud.h',##
        'model/uan-phy-mfsk.h',
        'model/uan-transducer-hd.h',
        'model/uan-prop-model-ideal.h',
        'model/uan-mac-aloha.h',
        ##'model/uan-mac-pure-aloha.h',##
        'model/uan-mac-new-aloha.h',##
        'model/uan-mac-maca.h',##
        'model/uan-mac-fc-maca.h',##
        'model/uan-emulation-packet.h',##
        'model/uan-emulation-manager.h',##
        'model/uan_tool.h',##
        'model/uan_byteOrder.h',##
        'model/uan-header-common.h',
        'model/uan-header-new.h',##
        'model/uan-header-fc.h',##
        'model/uan-noise-model.h',
        'model/uan-noise-model-default.h',
        'model/uan-mac-cw.h',
        'model/uan-prop-model-thorp.h',
        'model/uan-phy-dual.h',
        'model/uan-header-rc.h',
        'model/uan-mac-rc.h',
        'model/acoustic-modem-energy-model.h',
        'helper/uan-helper.h',
        'helper/acoustic-modem-energy-model-helper.h',
        'model/uan-mac-rc-gw.h',
        ]

    if (bld.env['ENABLE_EXAMPLES']):
      bld.recurse('examples')

    bld.ns3_python_bindings()
