
# Driving
- courant nominal: 4 130 mA
- thermal time constante winding: 25.1 s
- max speed: 10 000 (avec 8 pole 6 250)
- pole: 8
- reduction: 53.1

sensor: hall senor
    type: bloc communication

regulation: main sensor: hall sensor
following error windows: mettre max possible (20000....)


# Steering
- courant nomila: 1 880 mA
- thermal time cte windinw: 16.5s
- max speed: 12 500
- pair de poles: 4
- reduction: 729

- inverse pour la direction de la rotation
- sensors: 
    hall sensor,
    encode sur le moteur shaft (digital incrementatal encoder)
    ssi sur le gear
    
encoder incremental: 
    number pulse: 1024 par resolution

- commutation: sinusoidale
X4 (hall sensor) et X5 (encoder incremental)
- main sensor: encoder incremental


# Motor Reduction
- un tour c'est 2¹⁷ 
- 


# Extern encoder (absolute encoder)
- tuned for 5 rpm
