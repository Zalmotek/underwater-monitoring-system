# underwater-monitoring-system

This repository contains all the resources used in the development process of an underwater surveillance and ocean health monitoring system.

### System Architecture
![Schematic](/resources/DianaSchematic.jpg)  

The system of is comprised of an array of smart underwater buoys and a data collection device mounted on a vessel.

The buoys are distributed at fixed positions in a surface that must be monitored. These buoys are designed to monitor environmental data and, through machine learning on the edge models, detect and record anomalies.

The data collection device, which can be attached to a vessel or an array of buoys, acts as a gateway between the underwater and over-ground environments. It possesses the capability to identify underwater buoys using GPS and establish communication with them. By utilizing acoustic signals, the gateway device acquires data and then transmits it to a cloud-based solution. This solution enables data storage, interrogation, visualization, and the triggering of alarms.

### Contents
- [CAD](cad)
  
  This folder contains files and resources that are integral to both the conceptualization and realization stages of our product development. The specific items within this folder include:
- [Hardware](hardware)
  
This folder contains the KiCAD Project, Schematics & Layout and Project Libraries.

- [Firmware](firmware)
  
This folder hosts firmware samples developed for the underwater monitoring system.

- [Docs](docs)
  
  This folder contains Data Sheets and documentation for the project

- [Production](production)
  
This folder contains the gerber files, BOM or anything required for fabrication.

- [Resources](resources)
  
  This folder hosts pictures and schematics used in the Readme files.
