P a g e \| **1**

> INCREASING THE EFFICIENCY OF REGENERATIVE BRAKING USING AN ALGORITHM
>
> **Jason Royan, Guthula Ravi Shankar, Vasanth Balaji, \* Dr. Ramesh
> Kumar. C**
>
> **[Abstract]{.ul}**

A regenerative braking algorithm is proposed to make maximum use of
regenerative brake for reducing fuel consumption. In the regenerative
braking algorithm, the regenerative torque is determined by considering
the motor capacity, battery SOC and vehicle velocity. The regenerative
braking force is calculated from the brake control unit by comparing the
demanded braking torque and the motoring torque available. A
motor/generator is coupled to the front axle to perform regenerative
braking at high speeds. On the rear wheel, a Kinetic Energy Recovery
System (KERS) has been implemented using a ring flywheel to recover
energy from the rear axle at low speeds. Based on the motor
characteristics, the designed algorithm will assess the dynamic
distribution strategy for maximum use of regenerative braking. The
results show that the proposed algorithm can successfully recover and
store energy at high and low speeds, thereby improving performance and
energy efficiency.

> **[Keywords]{.ul}:** regenerative braking, blending control,
> algorithm, energy recovery, KERS, motor/generator
>
> **1. [Introduction]{.ul}**
>
> There has been an increasing trend and focus towards the electric
> vehicle, as there is a rise in the awareness towards the environmental
> impact of the decisions one takes. Keeping this in mind many are
> opting for electric or hybrid electric vehicles for their commute and
> modes of transportation. When a vehicle is moving it has a certain
> amount of kinetic energy, this energy is lost to the environment in
> conventional braking systems as heat. A successfully designed braking
> system for a vehicle must always meet two distinct demands:
>
> i\. In emergency braking, the vehicle should be able to come to rest
> in the shortest possible distance.
>
> ii\. The braking system should provide direction stability, which
> requires refined control on the braking force distribution on all
> wheels.
>
> In EV's and HEV's, instead of conventional braking; regenerative
> braking is being employed which is able to harvest a fraction of the
> energy being dissipated as heat in a conventional braking system. In
> their most fundamental form, they are a bi-directional power
> transmission system, with a power source and sink at one end, and an
> energy storage device on the other. As of now the systems are able to
> recover only up to 30-35% of the energy. But this is a region of study
> that is attracting a lot of attention as it has a lot of potential.
> There are various systems to recover energy: electrical, mechanical
> and hydraulic.
>
> **[Electrical regeneration properties]{.ul}**
>
> Electrical energy recovery systems are at their core, based on a
> motor/generator driven from the kinetic energy of the moving vehicle.
> The electricity generated is used to charge a battery, and then fed
> back to the machine in the acceleration phase. The energy is
> transformed from kinetic energy to electrical energy, and then to
> chemical energy in the regenerative braking phase. These
> transformations occur in reverse during acceleration. Due to the large
> number of energy conversions, electrical regeneration has a relatively
> poor round-trip efficiency, even in the most efficient systems. The
> specific power (kW/kg) of such systems is very highly dependent on the
> size and chemistry of the battery, but is typically much poorer than
> both hydraulic and mechanical systems. However, the specific energy
> storage is much better than hydraulic systems, and the main benefit of
> a fully-electric system lies in the packaging freedom that the
> designer is allowed. Electrical regenerative braking may be
> implemented in any electric, or part-electric vehicle. In fully
> electric vehicles, the power flow is reversed through the entire
> powertrain, whereas in part-electric vehicles power flow is reversed
> through only the electric portion of the [powertrain.]{.ul}
> **[Mechanical regeneration properties]{.ul}**
>
> Mechanical regeneration (known as M-KERS, or Mechanical Kinetic Energy
> Recovery System) is based on a flywheel and clutch system, in which a
> flywheel is clutched into the driveline at the beginning of a braking
> event, and the momentum of the vehicle is used to accelerate the
> flywheel, thereby slowing the vehicle down. The reverse is true in the
> acceleration phase, where the momentum of the flywheel is used to aid
> acceleration. This mechanical energy storage has advantages over other
> types of regeneration. Its key advantage is that by mechanically
> coupling a flywheel to the already-spinning driveline, there are no
> energy conversion losses. Further, as an energy storage medium and
> power delivery medium, it also boasts far better energy density
> (kJ/kg), and power density (kW/kg) than the other types of
> regenerative systems.
>
> **[Fluid-based regeneration properties]{.ul}**
>
> Fluid-based regeneration is either pneumatic or hydraulic, although
> due to the compressibility of air, pneumatic braking is limited to the
> use of retarders, which are non-regenerating devices. A hydraulic
> regenerative braking system utilizes a low-pressure reservoir, a
> hydraulic pump/motor, and high pressure accumulators to store energy
> in compressed fluid. Under braking, the pump is powered from the
> driven shaft and pumps fluid from the low
>
> pressure reservoir to the high-pressure accumulator, which is charged
> with an inert gas to exert pressure on the working fluid. The
> pressurized fluid is then used in the motoring phase by reversing the
> flow, allowing the fluid to flow through a hydraulic motor (in some
> cases the pump doubles as the motor), which is connected to the
> powertrain at a convenient location.
>
> **[Blended-Braking Energy Management]{.ul}**
>
> The task of braking energy management is to explore the potential of
> using regenerative braking to the maximum extent by reasonably
> allocating the regenerative braking force and friction braking force,
> improving the energy efficiency of an electric vehicle (EV) as much as
> possible. In regenerative braking control, currently available
> research mainly concentrates on a normal deceleration process with the
> aim of improving the regeneration efficiency and coordinated control
> between the regenerative brake and the frictional brake.
>
> As the vehicles are mostly front engine, front wheel drive the weight
> distribution is such that there is more weight on the front axle,
> which becomes even more prominent while braking, as there is a dynamic
> load shift towards the front axle. Thus, taking this into
> consideration the regeneration is done from the front axle.
>
> Factors to be considered during regeneration are:
>
> • State of Charge of the Battery.
>
> • Vehicle Speed.
>
> • Low Speed Cutoff Point.
>
> • Rate of Deceleration.
>
> • Wheel locking.
![This is an image](/media/image1.png)
![This is an image](/media/image2.png)


>   **Figure.2.** Regenerative
> Motor/Generator Flowchart
>
> **2. [Methodology]{.ul}**
>
> Our target is to improve the efficiency of the regenerative system so
> that we can improve the energy recovery and at the same time increase
> the life cycle of conventional braking system and reduce the
> dependency on Conventional braking.
>
> Our approach is to add a flywheel at the rear axle, increasing the
> weight at the rear end and thus increasing the locking force which
> allows for a comparatively greater energy recovery, while not majorly
> disturbing other factors like: rate of deceleration, stopping distance
> and time.
> **[Design of Flywheel]{.ul}**
>
> ● Kinetic Energy, KE =(½)\*I\*Ꞷ^2^
>
> ● Moment of Inertia, I =(½)\*M\*R^2^
>
> ● Mass of flywheel, M = 10 kgs
>
> ● Ꞷ = 314.159 rad/s
>
> ● RPM=3000
>
> ● I = 0.465125 kg.m2
>
> ● KE = 22.959 kJ

| W    | Wf  | Wr  | Fw    |
|------|-----|-----|-------|
| 1300 | 702 | 598 | 12740 |
| 1310 | 702 | 608 | 12838 |
| 1320 | 702 | 618 | 12936 |
| 1330 | 702 | 628 | 13034 |

> **Table 1: Weight Distribution on Axles**

Weight Distribution

![This is an image](/media/image6.png)
**Fig 3: Weight Distribution**

From figure 3. We see that upon adding a flywheel to the read axle we
are able to better the brake force distribution and reduce the
difference between the weight transfer between the wheels. By doing this
we reduce the load the motor generator has to carry while regenerating
by transferring some of that load to the rear axle to be use by the KERS
system.

| W    | L    | a        | b        |
|------|------|----------|----------|
| 1300 | 2.54 | 1.1684   | 1.3716   |
| 1310 | 2.54 | 1.17887  | 1.36113  |
| 1320 | 2.54 | 1.189182 | 1.350818 |
| 1330 | 2.54 | 1.199338 | 1.340662 |

> **Table 2: Centre of Gravity Location**


![This is an image](/media/image7.png)

> **D**
>
> **G**

**Weight**

> **C**
>
> Distance of CG from Front Axle Distance of CG from Rear Axle **Fig 4:
> Centre of Gravity**
>
> Similarly, from figure 4., we see the CG position is shifted
> backwards. The flywheel available at the rear axle allows some of the
> weight to be shifted from the front axle to the rear axle.

+---------------------+---------------------+---------+
| Fbf                 | Fbr                 | Fb      |
+=====================+=====================+=========+
| > 7148.845 7161.5   | > 3043.155 3108.9   | 10192   |
| >                   | >                   |         |
| > 7174.156 7186.811 | > 3174.644 3240.389 | 10270.4 |
|                     |                     |         |
|                     |                     | 10348.8 |
|                     |                     |         |
|                     |                     | 10427.2 |
+---------------------+---------------------+---------+

> **Table 3: Locking Force (Braking)**
>
> Brake force distribution vs Weight

**e**

**c**

**r**

**o**

**F**

**e**

**k**

**a**

**r**

**B**

8000 6000 4000 2000

7148.845354 7161.500472 7174.155591 7186.810709

3043.154646 3108.899528 3174.644409 3240.389291

1295 1300 1305 1310 1315 1320 1325 1330 1335 **Weight**

Fbf Fbr

**Fig 5: Braking Force wrt Weight**

+---------+---------+
| > a/g f | > a/g r |
+=========+=========+
| 0.82    | 0.82    |
|         |         |
| 0.82    | 0.82    |
|         |         |
| 0.82    | 0.82    |
|         |         |
| 0.82    | 0.82    |
+---------+---------+

**Table 4: Rate of Deacceleration**

P a g e \| **6**

+---------+---------+---------+---------+---------+---------+---------+
| Vel     | Vel m/s | KE      | s       | T       | ER      | PR      |
| km/hr   |         |         |         |         |         |         |
+=========+=========+=========+=========+=========+=========+=========+
| 30      | > 8     | > 4     | > 2     | > 3     | > 7     | 25.9075 |
|         | .333333 | 5486.11 | 5.42885 | .051463 | 9055.77 |         |
| 40      | > 1     | >       | > 2     | > 2     | > 8     | > 3     |
|         | 1.11111 | 80864.2 | 8.87352 | .598617 | 9764.89 | 4.54333 |
| 50      | > 1     | >       | > 3     | > 2     | > 1     | > 4     |
|         | 3.88889 | > 1     | 3.30237 | .397771 | 03533.8 | 3.17917 |
| 60      | > 1     | 26350.3 | > 3     | > 2     | > 1     | >       |
|         | 6.66667 | > 1     | 8.71542 | .322925 | 20362.4 |  51.815 |
| 70      | > 1     | 81944.4 | > 4     | > 2     | > 1     | >       |
|         | 9.44444 | > 2     | 5.11265 | .320079 | 40250.7 | > 6     |
| 80      | > 2     | 47646.6 | > 5     | > 2     | > 1     | 0.45083 |
|         | 2.22222 | > 3     | 2.49408 | .362234 | 63198.8 | > 4     |
| 90      | > 25    | 23456.8 | > 6     | > 2     | > 1     | 9.08667 |
|         |         | >       | 0.85969 | .434388 | 89206.7 | >       |
|         |         |  409375 |         |         |         | 77.7225 |
+---------+---------+---------+---------+---------+---------+---------+

> **Table 5: Value Table for Flywheel of 10kg.**
>
> KE, Stopping Distance and Time VS Speed
>
> 450
>
> 400
>
> 350
>
> 300

T

,

s

,

E

K

)

W

k

(

e

r

w

o

P

,

)

J

k

(

y

g

r

e

n

E

250

200

150

100

50

0

0 20 40 60 80 100 Speed

KE s T

**Fig 6: KE, Stopping Distance and Time VS Speed**

Energy, Power Recoverable VS Speed

200

150

100

50

0

0 20 40 60 80 100 Speed

ER PR

**Fig 7: Energy, Power Recoverable VS Speed**

> The reason we chose a ring type flywheel instead of the disc type of
> flywheel is because of the energy density of the ring type flywheel

P a g e \| **7**

> ![](vertopal_b8410a002b4f4e4ea75a18e1748dfa9f/media/image3.png){width="4.766666666666667in"
> height="2.6333333333333333in"}**Figure 5:** Flywheel KE
>
> **[Regenerative Braking Algorithm]{.ul}**
>
> This algorithm gets its inputs from the speed sensors and brake pedal
> position, rpm of motor, ultra-capacitor, voltage, it calculates the
> maximum regenerative torque from the inputs, it calculates the rate of
> deceleration, wheel lockup force, braking force required compares it
> with the available regenerative braking force, and if insufficient it
> will supply the balance force using the conventional hydraulic
> braking.
>
> **[Algorithm for regenerative braking using C++]{.ul}**

\#include\<iostream\>

> using namespace std;
>
> int main()
>
> {

int Tem,Tumax,Tem_r,Pge_m,Pch_m,nem;

Pge_m=3000;

Pch_m=20000;

cout\<\<\"RPM of Electric Motor: \";

> cin\>\>nem;
>
> if(nem\<=1500)

Tem=Pge_m\*9550/1500;

else

> Tem=Pge_m\*9550/nem;

cout\<\<\"Torque of Motor: \"\<\<Tem\<\<\"\\n\";

> Tumax=Pch_m\*9550/nem;

cout\<\<\"Torque from Ultracapacitor: \"\<\<Tumax\<\<\"\\n\";

int w1,u,vs;

float w2,wb;

cout\<\<\"U: \";

> cin\>\>u;
>
> if(u\>=30 && u\<46)
>
> w1=1;

P a g e \| **8**

> else if(u\>=46 && u\<48)
>
> w1=-0.5\*u+24;
>
> else
>
> w1=0;

cout\<\<\"\\nWeight factor of UC: \"\<\<w1\<\<\"\\n\";

cout\<\<\"Vehicle speed: \";

cin\>\>vs;

if(vs\<10){

> w2=0;
>
> }

else if(vs\>=10 && vs\<30){

w2=0.05\*vs-0.5;

> }

else

w2=1;

cout\<\<\"Weight factor of speed: \"\<\<w2\<\<\"\\n\";

> wb=(float)w1\*w2;
>
> cout\<\<\"Weight factor: \"\<\<wb\<\<\"\\n\";

if(Tumax\<Tem\*wb)

Tem_r=Tumax;

else

Tem_r=Tem\*wb;

cout\<\<\"Torque Regenerative: \"\<\<Tem_r\<\<\"\\n\";

float Freg,r,Fb,Fflock,Frlock,mu,Fdem,F,Fh,Fhyd;

float z,vsf,vsi,dt,m,g,a,b,h,W,Wf,Wr,s,Er,Eb,KE;

;

r=0.4;

Freg=Tem_r/r;

cout\<\<\"Force Regenertive: \"\<\<Freg\<\<\"\\n\";

cout\<\<\"Velocity at braking: \";

cin\>\>vsf;

cout\<\<\"\\nVelocity after braking: \";

cin\>\>vsi;

a=1.198;

b=1.342;

h=0.5;

m=1310;

W=12838;

g=9.81;

dt=1;

z=(vsf-vsi)/dt/g;

Wf=b/(a+b)\*W+h/(a+b)\*z\*W;

Wr=a/(a+b)\*W+h/(a+b)\*z\*W;

mu=2.943;

Fflock=mu\*Wf;

P a g e \| **9**

Frlock=mu\*Wr;

Fb=Fflock+Frlock;

cout\<\<\"Front Locking force: \"\<\<Fflock\<\<\"\\n\";

cout\<\<\"Rear locking force: \"\<\<Frlock\<\<\"\\n\";

cout\<\<\"Total braking force: \"\<\<Fb\<\<\"\\n\";

KE=.5\*m\*vsi\*vsi;

s=KE/Fb+21;

Er=Frlock\*s;

Eb=Er-45918;

Fh=Eb/s;

cout\<\<\"Demanded Brake force: \";

cin\>\>Fdem;

if(Fdem\>Fflock)

F=Fdem;

else

F=Fflock;

cout\<\<\"\\nBrake Force \"\<\<F\<\<\"\\n\";

if(F\>Freg)

Fhyd=F-Freg;

else

Fhyd=0;

cout\<\<\"Required Hydraulic Brake force:\"\<\<Fhyd\<\<\"\\n\";

return 0;

> }
>
> **3. [SIMULINK Model for Flywheel System]{.ul}**
>
> A SIMULINK model has been constructed to plot the change in velocity
> as the KERS system is activated. A vehicle body block was used to
> measure the longitudinal velocity. A quarter car model was adopted for
> modelling KERS system.
>
> ![](vertopal_b8410a002b4f4e4ea75a18e1748dfa9f/media/image4.png){width="6.326388888888889in"
> height="3.1729166666666666in"}Fig. 6 -- Model of KERS System using
> Simulink

P a g e \| **10**

> An inertia block represents the flywheel coupled at the rear axle. We
> have used Tresca's Magic tire formula while giving the vehicle's
> normal reaction with respect to ground to the tire block. Slip factor
> is neglected as the KERS system will be operated at low speeds where
> slip does not play a roll. Similarly the horizontal wind force can
> also be neglected. A Solver Configuration bloc is used to excite the
> system. As we cannot measure the rotational component using a scope,
> the output velocity from the vehicle is plotted using a scope via PS
>
> Simulink converter .

![](vertopal_b8410a002b4f4e4ea75a18e1748dfa9f/media/image5.png){width="2.920138888888889in"
height="2.0658333333333334in"}

Fig. 7 -- Output of velocity

> From the output graph we can see that velocity gradually decreases
> over time. The reason why the velocity returns negative is because
> after a point, the KERS system begins to transfer some of the speed
> back to the axle and roles are reversed. To avoid this, a clutch
> system needs to be integrated into the design.
>
> **4. [Results and Discussion]{.ul}**
>
> After calculating various conditions and speeds we can say that adding
> a flywheel doesn't drastically affect the performance and energy
> requirements. Instead it increases the wheel lock up force thus
> allowing to extract more energy from the rear axle. The main reason
> for extracting energy only from the front axle is because during
> braking there is a dynamic load shift towards the front axle
>
> **5. [Conclusions]{.ul}**
>
> • Our project looks into the brake force distribution and its effects
> of power regeneration from the front and rear wheels using an
> Algorithm and a Simulink model.
>
> • At low speeds, the KERS system stores power whereas the motor is in
> idle condition and doesn't engage in regeneration as there is more
> power lost than generated.
>
> • At high speeds, the motor/generator will regenerate more power as
> 60% of vehicle weight is transferred to the front axle, enabling for
> optimum braking and regeneration by applying the algorithm. • The KERS
> system is also applicable at higher speeds but only up to a certain
> limit where slip does not occur and deceleration is not exceeding
> 0.3g. At this speed range, the motor/generator combined with the KERS
> system can maximize regeneration simultaneously applying sufficient
> braking to both wheels.
>
> **6. [References]{.ul}**
>
> 1\. Regenerative Braking Algorithm for an ISG HEV Based on
> Regenerative Torque Optimization XIAO Wen-yong1 , WANG Feng∗ , ZHUO
> Bin (Institute of Auto Electronic Technology, School of Mechanical
> Engineering, Shanghai Jiaotong University, Shanghai 200240, China
>
> 2\. Brake-Blending Control of EVs Chen Lv\*, Hong Wong\* and Dongpu
> Cao\*
>
> 3\. Integration and Performance of Regenerative braking and Energy
> Recovery Technologies in Vehicle P. Tawadros\* and N. Zhang\*
>
> 4\. Influencing Factors in Low Speed Regenerative Braking Performance
> of Electric Vehicles Shoeib Heydari\*, Poria Fajri\*, Nima Lotfi\*\*,
> Bamdad Falahati.
>
> 5\. Regenerative Braking Control Algorithm for an Electrified Vehicle
> Equipped with a By-Wire Brake System Chen Lv, Junzhi Zhang, Yutong Li,
> and Ye Yuan
>
> 6\. A Control Algorithm for the Novel Regenerative--Mechanical Coupled
> Brake System with by-Wire Based on Multidisciplinary Design
> Optimization for an Electric Vehicle Changran He \*, Guoye Wang,
> Zhangpeng Gong, Zhichao Xing and Dongxin Xu
