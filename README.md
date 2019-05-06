# MultilaterationTDOA
This library will be used for TDoA localization from a [Bitcraze loco positioning system](https://www.bitcraze.io/loco-pos-system/).

This library fits your use-case if:

* **Setup**:
    * Multiple fixd anchors doing TWR between them
    * Tags receiving data from the anchors and computing the Time Difference on Arrival of messages between the messages of Anchors
* **Measurements**:
    * Position anchor 1 (m)
    * Position anchor 2 (m)
    * TDoA (m, you can take the time divided by the speed of light for the conversion)

