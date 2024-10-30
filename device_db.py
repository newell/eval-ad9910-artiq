core_addr = "192.168.1.57"

device_db = {
    "core": {
        "type": "local",
        "module": "artiq.coredevice.core",
        "class": "Core",
        "arguments": {
            "host": core_addr,
            "ref_period": 1e-9,
            "target": "cortexa9",
        },
    },
    "core_log": {
        "type": "controller",
        "host": "::1",
        "port": 1068,
        "command": "aqctl_corelog -p {port} --bind {bind} " + core_addr,
    },
    "core_moninj": {
        "type": "controller",
        "host": "::1",
        "port_proxy": 1383,
        "port": 1384,
        "command": "aqctl_moninj_proxy --port-proxy {port_proxy} --port-control {port} --bind {bind} "
        + core_addr,
    },
    "core_analyzer": {
        "type": "controller",
        "host": "::1",
        "port_proxy": 1385,
        "port": 1386,
        "command": "aqctl_coreanalyzer_proxy --port-proxy {port_proxy} --port-control {port} --bind {bind} "
        + core_addr,
    },
    "core_cache": {
        "type": "local",
        "module": "artiq.coredevice.cache",
        "class": "CoreCache",
    },
    "core_dma": {"type": "local", "module": "artiq.coredevice.dma", "class": "CoreDMA"},
    "led0": {
        "type": "local",
        "module": "artiq.coredevice.ttl",
        "class": "TTLOut",
        "arguments": {"channel": 0},
    },
    "led1": {
        "type": "local",
        "module": "artiq.coredevice.ttl",
        "class": "TTLOut",
        "arguments": {"channel": 1},
    },
    "led0": {
        "type": "local",
        "module": "artiq.coredevice.ttl",
        "class": "TTLOut",
        "arguments": {"channel": 0},
    },
    "led1": {
        "type": "local",
        "module": "artiq.coredevice.ttl",
        "class": "TTLOut",
        "arguments": {"channel": 1},
    },
    "spi0": {
        "type": "local",
        "module": "artiq.coredevice.spi2",
        "class": "SPIMaster",
        "arguments": {"channel": 2},
    },
    "io_reset": {
        "type": "local",
        "module": "artiq.coredevice.ttl",
        "class": "TTLOut",
        "arguments": {"channel": 3},
    },
    "drovr": {
        "type": "local",
        "module": "artiq.coredevice.ttl",
        "class": "TTLInOut",
        "arguments": {"channel": 4},
    },
    "io_update": {
        "type": "local",
        "module": "artiq.coredevice.ttl",
        "class": "TTLOut",
        "arguments": {"channel": 5},
    },
    "profile0": {
        "type": "local",
        "module": "artiq.coredevice.ttl",
        "class": "TTLOut",
        "arguments": {"channel": 6},
    },
    "profile1": {
        "type": "local",
        "module": "artiq.coredevice.ttl",
        "class": "TTLOut",
        "arguments": {"channel": 7},
    },
    "profile2": {
        "type": "local",
        "module": "artiq.coredevice.ttl",
        "class": "TTLOut",
        "arguments": {"channel": 8},
    },
    "pll_lock": {
        "type": "local",
        "module": "artiq.coredevice.ttl",
        "class": "TTLInOut",
        "arguments": {"channel": 9},
    },
    "master_reset": {
        "type": "local",
        "module": "artiq.coredevice.ttl",
        "class": "TTLOut",
        "arguments": {"channel": 10},
    },
    "sync_smp_err": {
        "type": "local",
        "module": "artiq.coredevice.ttl",
        "class": "TTLInOut",
        "arguments": {"channel": 11},
    },
    "dds0": {
        "type": "local",
        "module": "artiq.coredevice.ad9910_basic",
        "class": "AD9910Basic",
        "arguments": {
            "spi_device": "spi0",
            "profile0_device": "profile0",
            "profile1_device": "profile1",
            "profile2_device": "profile2",
            "sync_smp_err_device": "sync_smp_err",
            "pll_lock_device": "pll_lock",
            "io_update_device": "io_update",
            "io_reset_device": "io_reset",
            "master_reset_device": "master_reset",
        },
    },
}
