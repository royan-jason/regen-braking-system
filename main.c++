#include <iostream>
using namespace std;
int main()
{
    int Tem, Tumax, Tem_r, Pge_m, Pch_m, nem;

    Pge_m = 3000;
    Pch_m = 20000;

    cout << "RPM of Electric Motor: ";
    cin >> nem;

    if (nem <= 1500)
        Tem = Pge_m * 9550 / 1500;
    else
        Tem = Pge_m * 9550 / nem;

    cout << "Torque of Motor: " << Tem << "\n";

    Tumax = Pch_m * 9550 / nem;

    cout << "Torque from Ultracapacitor: " << Tumax << "\n";

    int w1, u, vs;
    float w2, wb;

    cout << "U: ";
    cin >> u;

    if (u >= 30 && u < 46)
        w1 = 1;
    else if (u >= 46 && u < 48)
        w1 = -0.5 * u + 24;
    else
        w1 = 0;

    cout << "\nWeight factor of UC: " << w1 << "\n";
    cout << "Vehicle speed: ";
    cin >> vs;
    if (vs < 10)
    {
        w2 = 0;
    }
    else if (vs >= 10 && vs < 30)
    {
        w2 = 0.05 * vs - 0.5;
    }
    else
        w2 = 1;

    cout << "Weight factor of speed: " << w2 << "\n";

    wb = (float)w1 * w2;

    cout << "Weight factor: " << wb << "\n";

    if (Tumax < Tem * wb)
        Tem_r = Tumax;
    else
        Tem_r = Tem * wb;

    cout << "Torque Regenerative: " << Tem_r << "\n";
    float Freg, r, Fb, Fflock, Frlock, mu, Fdem, F, Fh, Fhyd;
    float z, vsf, vsi, dt, m, g, a, b, h, W, Wf, Wr, s, Er, Eb, KE;
    ;
    r = 0.4;

    Freg = Tem_r / r;

    cout << "Force Regenertive: " << Freg << "\n";
    cout << "Velocity at braking: ";
    cin >> vsf;
    cout << "\nVelocity after braking: ";
    cin >> vsi;

    a = 1.198;
    b = 1.342;
    h = 0.5;
    m = 1310;
    W = 12838;
    g = 9.81;
    dt = 1;
    z = (vsf - vsi) / dt / g;
    Wf = b / (a + b) * W + h / (a + b) * z * W;
    Wr = a / (a + b) * W + h / (a + b) * z * W;

    mu = 2.943;
    Fflock = mu * Wf;
    Frlock = mu * Wr;

    Fb = Fflock + Frlock;

    cout << "Front Locking force: " << Fflock << "\n";
    cout << "Rear locking force: " << Frlock << "\n";
    cout << "Total braking force: " << Fb << "\n";

    KE = .5 * m * vsi * vsi;
    s = KE / Fb + 21;
    Er = Frlock * s;
    Eb = Er - 45918;
    Fh = Eb / s;

    cout << "Demanded Brake force: ";
    cin >> Fdem;

    if (Fdem > Fflock)
        F = Fdem;
    else
        F = Fflock;

    cout << "\nBrake Force " << F << "\n";

    if (F > Freg)
        Fhyd = F - Freg;
    else
        Fhyd = 0;
    cout << "Required Hydraulic Brake force:" << Fhyd << "\n";

    return 0;
}
