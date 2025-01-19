public class Dynamics {
 private float diametro = height/17.5;
 private float circPerimetro;
 private float espessuraLinha   = 1; 
 private float atritoCinetico   = 0.2;
 private float atritoSuperficie = 1;
 private float g = 9.8;
 private float v = 0;
 
 // Dinâmica
 private float massa = 0;            // Em quilograma, massa total do robô
 private float m_c = 0.110;        // massa do robô sem as rodas e motores
 private float m_w = 0;            // massa de cada roda com atuador (motor)
 private float momentoInercia;    // kg*m², total
 private float I_c = 0.0002;      // momento de inércia em relação ao centro de massa
 private float I_w;               // momento de inércia de cada roda e motor em relação ao eixo das rodas
 private float I_m = 0;              // momento de inércia de cada roda e motor em relação ao diâmetro das rodas
 private float acel_linear = 0;        // m/s²
 private float acel_angular = 0;         // rad/s²
 private float energiaCineticaRotacao = 0; // Joules
 private float energiaCineticaLinear  = 0; // Joules
 private float velLinear = 0;           // Em metros por segundo
 private float w  = 0;                // Em radianos por segundo
 private float wL = 0;               // Em radianos por segundo; vel. angular esquerdo
 private float wR = 0;              // Em radianos por segundo; vel. angular direito
 private float torque_esquerdo = 0; // N*m
 private float torque_direito  = 0; // N*m
 private float R_roda = 0.015;      // Raio das rodas em metro
 private float L_roda = 0.132;    // Distância entre as rodas
 private float d_cm = 0;        // Distância do centro de massa do eixo das rodas em metros
 private int   dT = 1000;       // Em microssegundos
 private MotorDynamic motorLeft; // Modelagem dos motores
 private MotorDynamic motorRight; // Modelagem dos motores
 
 private float lastPosition;
 private float r2; // Em metros
 private float x;
 private float y;
 private float position = 0;
 private float xCenter;
 private float yCenter;
 private float distPercorrida = 0; // Em metros
 private float raio; // = height*0.4;
 private float pointPosition;
 private float dX   = 0;
 private float dY    = 0;
 private float xPoint = 0;
 private float yPoint = 0;
 private float rPoint = height*0.02;
 private float K      = 0.005; // 0.00045;
 private float k      = 0;
 private color cor     =    color(200, 10 , 10 );
 private color corFundo  =  color(255, 255, 255);
 private color pointColor = color(0  , 0  , 180);
 private float dif_dist = 0;

 Dynamics(float x, float y) {
   this.xCenter = x;
   this.yCenter = y;
   this.x = this.xCenter;
   this.y = this.yCenter - this.raio;
   this.setR2(0.1);
   this.circPerimetro = 2*PI*this.r2;
   this.pointPosition();
   
   // Dinâmica
   this.massa = m_c + 2*m_w;
   this.I_w = 2*m_w*(L_roda*L_roda);
   this.momentoInercia = I_c + m_c*(d_cm*d_cm) + I_w + 2*I_m;
   this.motorLeft  = new MotorDynamic();
   this.motorRight = new MotorDynamic();
   this.motorLeft.dt = motorRight.dt = dT/1000000.0; //dt != dT, em segundos e microssegundos respectivamente
   motorLeft.t_ate =  0; //0.0001;
   motorRight.t_ate = 0; //0.0005;
 }
 
 //////////////////////////// Dinâmica \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
 
 public void setPWM(float p1, float p2) {
    float t_max = 7.0; // Tensão máxima nos motores
    
    p1 = constrain(p1, -4095.0, 4095.0);
    p2 = constrain(p2, -4095.0, 4095.0);
    p1 = (p1/4095.0) * t_max;
    p2 = (p2/4095.0) * t_max;
    
    //if((millis()/10) % 2 == 0) println(motorLeft.w_m, motorRight.w_m); //println(p1, p2);
    
    //p1 = p2 = 1;
    setTensao(p1, p2);
    
    this.dX = velLinear*cos(position)*dT/1000000.0;
    this.dY = velLinear*sin(position)*dT/1000000.0;
    
    this.dif_dist = sqrt(dX*dX + dY*dY);
    if(velLinear < 0) dif_dist *= -1.0;
    this.distPercorrida += dif_dist;
    
    this.xCenter = xCenter + dX*k;
    this.yCenter = yCenter - dY*k;
    
    velAngular(w); // Em radianos por segundo
    
  }
 
 public void setTensao(float vl, float vr) {
   //vl = 1; vr = 1;
   motorLeft.setTensao(vl, wL);
   motorRight.setTensao(vr, wR);
   this.setTorques(motorLeft.t_m, motorRight.t_m);
   //if((millis()/10)%4 == 0) println(motorLeft.t_m);
   //this.setTorques(0.0045, 0.0045);
 }
 
 public void setTorques(float tL, float tR) {
   this.torque_esquerdo = tL;
   this.torque_direito  = tR;
   float dt = dT / 1000000.0; // Variação de tempo em segundos
   w += getAcel_angular() * dt;
   velLinear += getAcel_linear() * dt;
   set_wL();
   set_wR();
 }
 
 public float getAcel_angular() {
   acel_angular = ((torque_direito - torque_esquerdo)*(L_roda/R_roda) - m_c*d_cm*w*velLinear) / (momentoInercia + 2*I_w*(L_roda*L_roda)/(R_roda*R_roda));
   return acel_angular;
 }
 
 public float getAcel_linear() {
   acel_linear = ((torque_direito + torque_esquerdo) / R_roda + m_c*d_cm*(w*w)) / (massa + 2*I_w/(R_roda*R_roda));
   return acel_linear;
 }
 
 public float get_wL() {
   wL = (velLinear - w*L_roda) / R_roda;
   return wL;
 }
 
 public float get_wR() {
   wR = (velLinear + w*L_roda) / R_roda;
   return wR;
 }
 
 public void set_wL() {
   wL = (velLinear - w*L_roda) / R_roda;
 }
 
 public void set_wR() {
   wR = (velLinear + w*L_roda) / R_roda;
 }
 
 ///////////////////////#####################\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
 
 public void restart() {
   this.velLinear = 0;
   this.w = 0;
   this.motorLeft.restart();
   this.motorRight.restart();
 }

 // Unidades em radianos
 
 public float variacaoPosition(float atual, float anterior) {
   float variavel;
   if((atual - anterior) < -PI) {
     variavel = atual - anterior + 2*PI;
   } else if((atual - anterior) > PI){
     variavel = atual - anterior - 2*PI;
   } else variavel = atual - anterior;
   return variavel;
 }

 private float variacaoPosition() {
   float variavel;
   if((this.position - this.lastPosition) < -PI) {
     variavel = this.position - this.lastPosition + 2*PI;
   } else if((this.position - this.lastPosition) > PI){
     variavel = this.position - this.lastPosition - 2*PI;
   } else variavel = this.position - this.lastPosition;
   return variavel;
 }

 public void velTangencial() {
   if(this.lastPosition != this.position) {
     this.v = (variacaoPosition())*this.circPerimetro*1000000/(2*PI*dT);
     //println(this.v + " m/s");
     this.lastPosition = this.position;
   }
 }

 public void velAngular() {
   if(this.lastPosition != this.position) {
     this.w = (variacaoPosition())*1000000.0/ (float) dT;
     println(this.w + " º/s");
     this.lastPosition = this.position;
   }
 }
 
 public void velAngular(float vel) {
   //this.w = vel;
   float a = this.position + dT*vel/1000000.0;
   setPosition(a);
   posicaoRadians();
 }

 public float distancia(float x1, float y1, float x2, float y2) {
   return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
 }
 
 public void pointPosition() {
   double a = (this.xPoint - this.xCenter);
   double b = sqrt(pow((this.xPoint - this.xCenter), 2) + pow((this.yCenter - this.yPoint), 2));
   if(this.yPoint <= this.yCenter) this.pointPosition = (float)(Math.acos(a/b));
   else this.pointPosition = 2*PI - (float)(Math.acos(a/b));
   //println(this.pointPosition);
 }

 public void position() {
   double a = (this.x - this.xCenter);
   double b = sqrt(pow((this.x - this.xCenter), 2) + pow((this.yCenter - this.y), 2));
   if(this.y <= this.yCenter) this.position = (float)(Math.acos(a/b));
   else this.position = 2*PI - (float)(Math.acos(a/b));
   //println(this.position);
 }
 
 public void posicao() {
   position();
   float p = distancia(mouseX, mouseY, xCenter, yCenter)/raio;
   this.x = (mouseX - xCenter)/p + xCenter;
   this.y = (mouseY - yCenter)/p + yCenter;
 }

 public void posicaoRadians(float angulo) {
   this.x = this.raio*cos(angulo) + this.xCenter;
   this.y = this.yCenter - this.raio*sin(angulo);
   this.position = angulo;
   //println(angulo);
 }

 public void posicaoRadians() {
   posicaoRadians(this.position);
 }

 public void circulo() {
   stroke(60);
   strokeWeight(this.espessuraLinha);
   line(x, y, xCenter, yCenter);
   fill(0);
   strokeWeight(1);
   ellipse(xCenter, yCenter, diametro/2, diametro/2);
   noStroke();
   fill(cor);
   ellipse(x, y, diametro, diametro);
 }

 public void triangulo() {
   stroke(0);
   fill(255, 255, 0);
   float x1 = 0.6*this.raio*cos((this.position - 150)*PI/180) + this.xCenter;
   float x2 = this.raio*cos(this.position*PI/180) + this.xCenter;
   float x3 = 0.6*this.raio*cos((this.position + 150)*PI/180) + this.xCenter;
   float y1 = this.yCenter - 0.6*this.raio*sin((this.position - 150)*PI/180);
   float y2 = this.yCenter - this.raio*sin(this.position*PI/180);
   float y3 = this.yCenter - 0.6*this.raio*sin((this.position + 150)*PI/180);
   triangle(x1, y1, x2, y2, x3, y3);
 }

 public void ponto() {
   noStroke();
   fill(this.pointColor);
   ellipse(this.xPoint, this.yPoint, 2*rPoint, 2*rPoint);
 }
 
 // set and get
 
 public void setConstante(float k) {
   this.k = k;
   this.raio = 0.5*k;
   //this.momentoInercia = this.K*this.massa;
 }
 
 public void setDistPercorrida(float d) {
   this.distPercorrida = d;
 }
 
 public void setDistRodas(float d) {
   if(d > 0) L_roda = d;
 }
 
 public float getDistRodas() {
   return this.L_roda;
 }

 public void setFundo(color c) {
   this.corFundo = c;
 }

 public void setCenter(float x, float y) {
   this.xCenter = x;
   this.yCenter = y;
   pointPosition();
 }
 
 public void setVariacaoTempo(int t) {
   this.dT = t;
   this.motorLeft.dt = motorRight.dt = dT/1000000.0;
 }
 
 public void setPointCenter(float x, float y) {
   this.xPoint = x;
   this.yPoint = y;
   pointPosition();
 }

 public void setRaio(float r) {
   if(r > 0) {
     this.raio = r;
   }
 }

 public void setR2(float r) {
   if(r > 0) this.r2 = r;
 }

 public void setPosition(float angulo) {
   if(angulo > 2*PI || angulo < -2*PI) {
     angulo = angulo - ((int)(angulo/(2*PI)))*2*PI;
   }
   if(angulo < 0) angulo = 2*PI + angulo;
   this.position = angulo;
 }

 public void setLineEspessura(float esp) {
   if(esp >= 1) this.espessuraLinha = esp;
 }
 
 public float getLineEspessura() {
   return this.espessuraLinha;
 }
 
 public float getMassa() {
   return this.massa;
 }
 
 public float getMomentoDeInercia() {
   return this.K;
 }
 
 public float getGravidade() {
   return this.g;
 }

 public float getX() {
   return this.x;
 }

 public float getY() {
   return this.y;
 }

 public color getCorFundo() {
   return this.corFundo;
 }
 
 public float getDistPercorrida() {
   return this.distPercorrida;
 }
 
 public float getDx() {
   return this.dX;
 }
 
 public float getDy() {
   return this.dY;
 }

 public float getPointPosition() {
   return this.pointPosition;
 }
 
 public float getXPoint() {
   return this.xPoint;
 }
 
 public float getYPoint() {
   return this.yPoint;
 }

 public float getPosition() {
   return this.position;
 }

 public float getRaio() {
   return this.raio;
 }

 public float getVelAngular() {
   return this.w;
 }
 
 public float getVelLinear() {
   return this.velLinear;
 }

 public float getXCenter(){
   return this.xCenter;
 }

 public float getYCenter(){
   return this.yCenter;
 }
 
 public float get_dif_dist() {
   return dif_dist;
 }
}

public class MotorDynamic {
  float i_max = 1.5; // Corrente máxima em amperes
  float i_a = 0; // Corrente de armadura
  float r_a = 30; // Resistência de armadura
  float L_a = 0.005; // Indutância de armadura, H (henry)
  float e_a = 0; // fem
  float t_m = 0; // Torque do motor em kg*m
  float k_t = 0.008; // Constante de torque
  float k_b = 0.001; // Constante para a fem
  float N = 30;  // Redução do motor, 30:1 por exemplo
  float w_m = 0; // Rotação do motor sem caixa de redução, não da roda
  float v_m = 0; // Tensão sobre a roda
  float t_ate = 0; //0.001; // Atrito estático
  float k_atc = 0.00000005; // Atrito cinético
  float dt = 1.0;  // Variação de tempo em segundos
  
  MotorDynamic() {
  }
  
  public void restart() {
    i_a = 0; // Corrente de armadura
    e_a = 0; // fem
    w_m = 0; // Rotação do motor sem caixa de redução, não da roda
    v_m = 0; // Tensão roda esquerda
  }
  
  public void setWMotor(float w) {
    w_m = N*w;
  }
  
  public void setTensao(float v, float w) {
    this.setWMotor(w);
    //println(w + "\t" + v);
    this.v_m = v;
    this.setTorque();
  }
  
  public void setTorque() {
    float atrito = k_atc*w_m; // Torque causado por atrito cinético
    e_a = w_m*k_b;
    //float di_a = ((v_m - e_a) - r_a*i_a) * dt/L_a; // Só funciona se dt for pequeno, menor que 300 x 10^-6
    //i_a += di_a;
    i_a = (v_m - e_a) / r_a; // Em tempo real
    t_m = i_a*k_t*N;
    if(abs(t_m) < abs(t_ate)) t_m = 0;   // Torque em sentido contrário devido ao atrito estático
    else t_m += (t_m > 0) ? -t_ate : t_ate; // Perda de torque por atrito estático
    t_m += -atrito;                         // Perda de torque por atrito cinético
  }
}
