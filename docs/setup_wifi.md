# Configuración WiFi — Raspberry Pi

## Agregar una red WiFi
```bash
sudo nmcli device wifi connect "NOMBRE_RED" password "CONTRASEÑA"
```

## Redes configuradas
Agregar las redes que use el equipo:
```bash
sudo nmcli device wifi connect "HOTSPOT_CAROL" password "xxxx"
sudo nmcli device wifi connect "HOTSPOT_COMPAÑERO1" password "xxxx"
sudo nmcli device wifi connect "WIFI_FIUNA" password "xxxx"
```

## Ver redes guardadas
```bash
nmcli connection show
```

## Ver red activa
```bash
nmcli connection show --active
```

## Ver IP de la Pi
```bash
hostname -I
```
