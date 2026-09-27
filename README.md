# PS3 streaming reference

Reference source snapshot for my PS3-to-Moonlight streaming project. Just for reference, not maintained, and not intended as a reusable project or supported setup.

- `sunshine/`: Sunshine 2025.924.154138 integration patch and Magewell capture/encoding backend.
- `relay/`: Python input/Guide-button relays, UART helper, and USB controller profiles. (I'm working to get the PS button working over USB, this webMAN approach is a temporary workaround)
- `firmware/`: Pico 2 UART-to-USB firmware and build helpers.
