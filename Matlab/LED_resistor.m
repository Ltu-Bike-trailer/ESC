%% 36 V
clc
VSS = 36
Vled = 1.75
I = 2e-3
R=(VSS-Vled)/I
R_power = R*I^2*1000

Rreal = 17800
Ireal=(VSS-Vled)/Rreal*1000
%% 15 V
clc
VSS = 15
Vled = 1.75
I = 2e-3
R=(VSS-Vled)/I
R_power = R*I^2*1000

Rreal = 6650
Ireal=(VSS-Vled)/Rreal*1000
%% 3.3 V
clc
VSS = 3.3
Vled = 1.75
I = 2e-3
R=(VSS-Vled)/I
R_power = R*I^2*1000

Rreal = 787
Ireal=(VSS-Vled)/Rreal*1000
%% 10 V
clc
VSS = 10
Vled = 1.75
I = 2e-3
R=(VSS-Vled)/I
R_power = R*I^2*1000

Rreal = 787
Ireal=(VSS-Vled)/Rreal*1000


