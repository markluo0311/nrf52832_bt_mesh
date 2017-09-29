# Sample Data
## Mesh Packet #1

    AppKey          = 63964771734fbd76e3b40519d1d94a48
    NetKey          = 7dd7364cd842ad18c17c2b820c84c3d6
    IVindex         =                             0002
    NetworkID       = d908d7ca64726f7712711d028d88908e
    IVzero          = cacf88f9482517393f0680051cd2df8a
    NetworkIV       = 565269d2ef2f77fb1026d2a69da082e0
    PrivacyKey      = e2e00f4dc2e08135d235052ec7f63afa

    App_nonce       =   00 01 02 03 04 05 26 d2 a6 9d a0 82 e0
    App CMIC        = 8f617a19e97d7ec100e45aafcb39b598
    App CBlock1     = a0014301d96e046c28fbe21680b18bcd
    App B0          = 0900010203040526d2a69da082e00001
    App B1          = 01000000000000000000000000000000
    App X0          = 0511b58200f3b11d4c0c80dedaee8db0
    App X1          = 8ac0bd4723ec2d824ed61efa32f0f92b

    Net_nonce       =   07010203040526d2a69da082e0
    Net CMIC        = d8a4c67e476eb9333d704e67a06f6e01
    Net CBlock1     = 0aecabef7f75f2f83294c872893af5bb
    Net B0          = 0907010203040526d2a69da082e00007
    Net B1          = 0607a105a1c75e000000000000000000
    Net X0          = b1bcf868582321450bed635c517bbedd
    Net X1          = b3b47382c5707766ce153d0f2ccceed0

    Privacy Counter = 565269d2ef2f77fb100ceb0aeadeb2ac
    Privacy ECB     = 2e70ec704bf11da3e36868bdd6d58796

    IVI              40
    NID              0e
    IVINID           4e
    PRI                00
    FUT                00
    TTL                07
    PFT                07
    SEQ                  010203
    SRC                        0405
    DST                            0607
    Payload                            01
    AppPacket        4e070102030405060701

    encApp                             a1
    MICapp                               05a1c75e
    TransportPacket  4e0701020304050607a105a1c75e

    NetHeader          070102030405
    encDSTData                     0ceb0aeadeb2ac
    MICnet                                       6b10b5fc
    NetPacket        4e0701020304050ceb0aeadeb2ac6b10b5fc

    PrivacyHeader      2971ee734ff4

    Packet           4e2971ee734ff40ceb0aeadeb2ac6b10b5fc

## Mesh Packet #2
    AppKey          = f1a24abea9b86cd33380a24c4dfbe743
    NetKey          = efb2255e6422d330088e09bb015ed707
    EncryptionKey   = 6184f68e9b32b266001177ad09b6f47c
    PrivacyKey      = 7ce92469e04bc2cbb3e2816980b8ead0
    NetworkID       = bc16a7dcd0b5f529ff289a8c32188a3e
    IVindex         =                             0102
    IVzero          = 12ff65bd86a616ed0542d6a8720b7b6c
    NetworkIV       = cc70ab10f5e41d1dc304c0fc421ddc0e

    app_nonce       =   8008090a0b0c04c0fc421ddc0e
    App CMIC        = ea511d6385b3db0bdaf17247677c7c99
    App CBlock1     = 8a21d09e1255bae10a4663897c4b7701
    App B0          = 098008090a0b0c04c0fc421ddc0e0009
    App B1          = 80110102030405060700000000000000
    App X0          = 897ae3364c5ab7fef44d85cd4bf67728
    App X1          = 8477b4053dea9e812d39cd1373d786b1

    net_nonce       =   9208090a0b0c04c0fc421ddc0e
    Net CMIC        = c80a2633a88f3e4e1e3ab4fdef890382
    Net CBlock1     = f2e98cb9cbecbad590baf36db1e24b12
    Net B0          = 099208090a0b0c04c0fc421ddc0e000f
    Net B1          = 0d0e0a30d19c1151bfe70d6e26a96600
    Net X0          = dfcc2146508668dbcafd66f89913e6f0
    Net X1          = 2519b02feb121ded0caf24ae5b89dea2

    Privacy Counter = cc70ab10f5e41d1dc3ffe786891a70ab
    Privacy ECB     = ded5478b1a8b8b4a6e37dd11c4c585c1

    IVI              40
    NID              1e
    IVINID           5e
    PRI                80
    FUT                00
    TTL                12
    PFT                92
    SEQ                  08090a
    SRC                        0b0c
    DST                            0d0e
    Payload                            801101020304050607
    AppPacket        5e9208090a0b0c0d0e801101020304050607

    encApp                             0a30d19c1151bfe70d
    MICapp                                               6e26a966
    TransportPacket  5e9208090a0b0c0d0e0a30d19c1151bfe70d6e26a966

    NetHeader          9208090a0b0c
    encDSTData                     ffe786891a70ab842f5dfe03974b2d
    MICnet                                                       ed13961c
    NetPacket        5e9208090a0b0cffe786891a70ab842f5dfe03974b2ded13961c

    PrivacyHeader      4cdd4e811187

    Packet           5e4cdd4e811187ffe786891a70ab842f5dfe03974b2ded13961c

## Secure Network Broadcast Beacon
    NetKey          = 736d6172746d657368696e7465726f70
    IVIndex         = 0002
    KR              = 00

    NetworkID       = b4ed5b4f54064f82923b9d1b9fda679a
    EncryptionKey   = 6ce984f79e0983a8e581591efa151ff0
    TempCMAC        = a2f25e43ebef298ce21bd90043d018c4
    CMAC            = b4688baa4d08a3e066de8dfad10281f7

    MeshBeacon      = 1216d17f02123b9d1b9fda679a0002d10281f7
