# Glossary

| Term | Meaning |
|---|---|
| ATX | The Hesai lidar on IFS09: 116 channels, ~120° horizontal FOV, scanned by a rotating mirror. |
| Channel / ring | One of the 116 lasers. `ring` in the cloud is the channel index. |
| Block | One azimuth step inside a packet; each has a reading for every channel. |
| Correction file (angle correction) | Per-channel azimuth and elevation offsets, unique to each lidar. Fetched from the lidar at start-up. |
| DVPC | The driverless computer on the car (LattePanda) that runs the driver and the pipeline. |
| Firetime correction | Per-channel timing offset of each laser; the driver turns it into a small azimuth shift. |
| FOV | Field of view. |
| Frame | One full scan, ~100 ms; on the ATX one mirror face. |
| Mirror face (even / odd frame) | The ATX mirror has two faces; consecutive frames use alternating faces with slightly different geometry. |
| No-return point | A laser shot with no echo, published as (0,0,0). |
| Packet | One UDP datagram from the lidar: 2 blocks × 116 channels on the car. |
| PTC | Pandar TCP Commands: Hesai's configuration protocol on TCP port 9347. |
| PTP | Precision Time Protocol: syncs the lidar clock to the DVPC. |
| QoS | ROS 2 delivery settings. *Best effort* drops late messages; *reliable* retries and can block the sender. |
| Remake | SDK feature that places points in a fixed azimuth × elevation (or ring) grid. |
| Return mode | Which echo the lidar reports: strongest, last, or both (dual). |
| UDP protocol 4.7 | Hesai's packet format version for the ATX. |
