![Static Badge](https://img.shields.io/badge/Version-0.1-8001AA?style=flat)
[![GitHub License](https://img.shields.io/github/license/openair-collective/openair-epiphyte)](LICENSE.txt)
[![Carbon Crowd](https://img.shields.io/badge/Website-Carbon%20Crowd-FBE80A)](https://carboncrowd.cc/)
[![Hackster](https://img.shields.io/badge/Website-Hackster-2E9FE6)](https://www.hackster.io/epiphyte/epiphyte-open-direct-air-capture-d55a9e)
[![GitHub Issues or Pull Requests](https://img.shields.io/github/issues/openair-collective/openair-epiphyte)](https://github.com/openair-collective/openair-epiphyte/issues)
[![GitHub forks](https://img.shields.io/github/forks/openair-collective/openair-epiphyte?style=flat)](https://github.com/openair-collective/openair-epiphyte/forks)
[![GitHub Repo stars](https://img.shields.io/github/stars/openair-collective/openair-epiphyte?style=flat&color=8001AA)](https://github.com/openair-collective/openair-epiphyte/stargazers)
[![GitHub watchers](https://img.shields.io/github/watchers/openair-collective/openair-epiphyte?style=flat&color=0CBAEC)](https://github.com/openair-collective/openair-epiphyte/watchers)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.13147932.svg)](https://doi.org/10.5281/zenodo.13147932)

# Epiphyte

## Introduction

Epiphyte is a miniature, stand-alone, open source direct air capture machine that removes CO2 from the air, designed by OpenAir volunteers.

The name "Epiphyte" comes from the same term referring to a plant that grows on the surface of another plant. The "host plant" here is Thursday ([YouTube intro](https://www.youtube.com/watch?v=E_M0lsxscfE)), a previous generation carbon capture device designed by the team at Octavia Carbon of Kenya.

There are two main goals of the Epiphyte device:

1. __To reproduce Thursday in an alternate form in order to provide sufficient documentation to rebuild such a device.__ Thursday is an older generation carbon capture device. It is no longer in active development, and was to be used as an example to propagate build iterations in other locations. However, parts of Thursday still could not be disclosed, and the purpose of the Epiphyte build was to untangle very technical details of Thursday so that clear documentation can be created for next generation Thursday builders.

1. __To create a modular venue for testing sorbents.__ Although not a major purpose of Epiphyte, the recipe for Thursday cannot be disclosed exactly. Epiphyte is built so that the sorbent could be swapped out to study the behavior of different ones, if needed.

For further background information see the [links section](#links).

### Current state of the work

The original Epiphyte was built in Philadelphia, and University of Pennsylvania is home to it. The next potential steps depend on the data collected from this device, including:
- Heating and temperature control and feedback
- Carbon dioxide sensing--whether the current sensors are adequate
- Use of basic sorbents

From this data, knowledge of the mechanics of the device can be extracted. Depending on the ultimate goal of the device, i.e., research vs commercial vs "maker-ware", Epiphyte can be further improved, or elements of it can be borrowed in a completely new device.

## About this repository

This repository attempts to collect all relevant design files, documentation, and links, and presents them as a package of material that can be used to gain a viable amount of knowledge toward understanding the mechanics of Thursday.

The root folder has the following subdirectories:

```sh
.
├── hardware_files/
├── license_info/
├── presentations/
└── software/
```

### `presentations`

The PowerPoint presentations here include background information and schematics from throughout the build process. Schematics of the entire built system, photos of the build process, as well as preliminary data can be found throughout the slides.

If looking for an introduction to Epiphyte, looking at these presentations first is recommended.

### `license_info` and `LICENSE.txt`

The files in the folder explain how to use the material in this repository for further development. The technology is open-sourced (license text), but a few instructions (`license_info/cern_ohl_p_v2_howto.pdf`) need to be followed for material reuse.

### `software`

Includes Arduino code for controlling heating and reading from sensors on Epiphyte.

### `hardware`

This section has bills of materials (BOM) as well as a list of 3D components that can be used to create 3D assemblies. See the [hardware README](hardware_files/README.md) for more details and supplemental links.


## Links

#### YouTube videos

- [About Octavia Carbon's Thursday](https://www.youtube.com/watch?v=E_M0lsxscfE)
- [Live walkthrough of Thursday](https://www.youtube.com/watch?v=BNXJqeISVzQ)
- Epiphyte webinars
    - [Episode 1 - Intro and build launch](https://www.youtube.com/watch?v=B53BWDbAE4Q)
    - [Episode 2 - Assembly report](https://www.youtube.com/watch?v=-oX7TFdO3ws)
    - [Episode 3 - Sorbent panel progress](https://www.youtube.com/watch?v=SeYeDm23_dI)
    - [Epiphyte @ COP28](https://www.youtube.com/watch?v=A-t8T-azwP8)

#### Other

- [Epiphyte on OpenAir forums](https://www.openairforum.org/c/rd/epiphyte/106)
- [Epiphyte on Hackster.io](https://www.hackster.io/epiphyte/epiphyte-open-direct-air-capture-d55a9e)
- [First build on Carbon Crowd website](https://carboncrowd.cc/builds/vvRHB4d3vJyN1Ihpj5yi) 
