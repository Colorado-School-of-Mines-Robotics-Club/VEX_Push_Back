# Wiring setup

## Parts

- https://www.pishop.us/product/dc-dc-ldo-5v-dc-dc-converter-5v-1-2a-output-6-5v-to-15v-dc-input/
- https://www.pishop.us/product/max-485-module-rs-485-module-ttl-to-rs-485-converter-module/

## Wiring

### MAX 485 converter

| Converter pin | Purpose        | Destination |
| ------------- | -------------- | ----------- |
| RO            | RS485 Recieve  | RPI UART RX |
| DI            | RS485 Send     | RPI UART TX |
| DE            | Driver enable  | RPI GPIO 18 |
| RE            | Recieve enable | RPI GPIO 17 |
