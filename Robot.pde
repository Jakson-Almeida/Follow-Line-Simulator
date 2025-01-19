public class Movimento {
  private Dynamics bot;
  private Button b1, b2;
  private int   startTime = 0;
  private float vC = 0;
  private float raio; // Raio entre as rodas e centro do robô em metros
  private float d_sensors = 0.09;
  private float Q = 0.032;
  private float S = 0.095;
  private float distRodas;
  private float diametroRoda = 0.03; // metros
  private float pRat = 6.2;
  private float velEncEsq = 0;
  private float velEncDir = 0;
  private float PWMLeft  = 0;
  private float PWMRight = 0;
  private float speedMotorLeft  = 0;
  private float speedMotorRight = 0;
  private int   cont_control = 0;
  private boolean Si = true;
  private boolean allow_noise = false;
  private float aleatorio1 = random(0, 100);
  private float aleatorio2 = random(200, 300); 
  private float xC;
  private float yC;
  private float lastXC = 0;
  private float lastYC = 0;
  private float lastAngle = 0;
  private float distPercorrida = 0;
  private int   numOscilacoes = 0;
  private float dX = 0;
  private float dY = 0;
  private float k = 1;
  private float w;
  private float h;
  private float v1 = 0;
  private float v2 = 0;
  private boolean som = false;
  private long time = 0;
  private long timeEsq = 0;
  private long timeMicros = 0;
  private int   dT = 1000; // Em microssegundos
  private int   comp;
  private short coord[][];
  private short t = 200;
  private PImage ratao;
  private boolean mode = true;
  private float vel_max_robo = 0; // Velocidade máxima em m/s
  private float Kp = 0;
  private float Ki = 0;
  private float Kd = 0;
  private int   vel = 0;
  private int   erro = 0;
  private float lastKp = 0;
  private float lastKi = 0;
  private float lastKd = 0;
  private int   lastVel = 0;
  public  ArrayList<Control> controles = new ArrayList<Control>();
  private int  last_proportional = 0;
  private long erroAbsoluto = 0;
  private long erroAbsolutoQuadrado = 0;
  private long integral = 0;
  private float varredura = 44.0;
  private short numSensors = 5;
  private float coordSensors  [][] = new float[numSensors][2]; // 0 -> x, 1 -> y
  private float lateralSensors[][] = new float[2][2];
  private int sensors[] = new int[numSensors]; // Leitura
  private int value  [] = new int[numSensors];
  private char mat[][];
  private int contEsq = 0;
  private int contDir = 0;
  private boolean lastEsq = false;
  private boolean flagEsq = false;
  private boolean lastDir = false;
  private boolean flagDir = false;
  private boolean cruzamento = false;
  private boolean saiuDaLinha = false;
  private int     erroMaximo = 20; //250000;
  private short   linha = 128;
  private boolean sobreSensorEsq = false;
  private boolean sobreSensorDir = false;
  private boolean considerInertia = true;
  private int[]   values = {0, 0, 0, 0};
  private String  recebeDados;
  private float   fitness = 0;
  private String  atualiza = "";
  private int     amplitude = 1000000;
  private int    d_error = 0;
  private float angle_max = 0.3; // Em radianos
  private float angle_integrative = 0.15; // Maior ângulo absoluto em que o integrativo atua
  private int   fator = 1;
  private float ts = (dT / 1000000) * fator;
  private float tf = 1.824;
  private float last_error = 0;
  private float last_derivative = 0;
  private float k_m1 = 1.0; // Constantes de eficiência dos motores do robô
  private float k_m2 = 1.0; // Os valores estão no intervalo de 0 a 1
  private float vel_linear = 0.3;
  private float initialSpeed = vel_linear;
  private float dist_atual = 0;
  private float dist_esq = 0;
  private float dist_dir = 0;
  private float dist_final = 0;
  private float dist_rot = 0;
  private int   last_index = 0;
  private int   last_index2 = 0;
  private float radius = 1000000;
  private float dif_dist = 0;
  private float last_kp = 0;
  private float last_ki = 0;
  private float last_kd = 0;
  private int   somaErros = 0;
  private float last_dis = 0;
  private long  first_right_time = 0;
  private long  final_right_time = 1000000000;
  private int contagemMaxima = 16;
  private boolean printLateralSensors = false;
  public  Control malha_ext_esq;
  public  Control malha_ext_dir;
  private SimpleKalmanFilter kalman = new SimpleKalmanFilter(2000, 400, 3);
  private Neural_Network neuron = new Neural_Network(1, 1, 1);
  private Matrix vetEntrance = new Matrix(1, 1);
  private float[] neuronOutput = new float[2];
  private float[][] intervalo = {{ 0, 0.15  },
                                 { 0, 0.001 },
                                 { 0, 10.0  },
                                 { 0, 4095  }};
  
  Movimento(float xCenter, float yCenter, float angle, float constante, boolean inercia) {
    this.ratao = loadImage("Ratão 4.0.png");
    imageMode(CENTER);
    this.coord = new short[t][2];
    this.bot = new Dynamics(xCenter, yCenter);
    this.bot.setPosition(angle);
    for(short i = 0; i < t; i++) {
      this.coord[i][0] = (short) bot.getXCenter();
      this.coord[i][1] = (short) bot.getYCenter();
    }
    //this.b1 = new Button(0.6*width, 0.7*height, 0.3*width, 0.08*height);
    //this.b2 = new Button(0.6*width, 0.85*heighFFt, 0.3*width, 0.08*height);
    //this.b1.setCor(color (0, 50, 250));
    //this.b2.setCor(color (0, 200, 50));
    this.k = abs(constante); // Pixels/metro
    this.bot.setConstante(k);
    this.xC = bot.getXCenter();
    this.yC = bot.getYCenter();
    this.comp = ceil(height*0.15);
    this.h = comp/sqrt(1.0 + (width*width)/( height*height ));
    this.w = h*width/height;
    this.considerInertia = inercia;
    this.distRodas = bot.getDistRodas();
    this.raio = bot.getDistRodas() / 2.0;
  }
  
  // Calibracao
  
  private void frontSensors() { // numSensors ímpar
    int   centro = (numSensors+1)/2 - 1;
    float angle  = bot.getPosition();
    this.coordSensors[centro][0] = bot.getXCenter() + k*d_sensors*cos(angle);
    this.coordSensors[centro][1] = bot.getYCenter() - k*d_sensors*sin(angle);
    for(short i = 1; i < centro+1; i++) {
      coordSensors[centro + i][0] = coordSensors[centro][0] - i*k*Q*cos(angle+PI/2f)/numSensors;
      coordSensors[centro + i][1] = coordSensors[centro][1] + i*k*Q*sin(angle+PI/2f)/numSensors;
      coordSensors[centro - i][0] = coordSensors[centro][0] + i*k*Q*cos(angle+PI/2f)/numSensors;
      coordSensors[centro - i][1] = coordSensors[centro][1] - i*k*Q*sin(angle+PI/2f)/numSensors; 
    }
  }
  
  private void lateralSensors() {
    float da = 0.72;
    float angle = bot.getPosition();
    this.lateralSensors[0][0] = bot.getXCenter() + k*S*cos(angle + da);
    this.lateralSensors[0][1] = bot.getYCenter() - k*S*sin(angle + da);
    this.lateralSensors[1][0] = bot.getXCenter() + k*S*cos(angle - da);
    this.lateralSensors[1][1] = bot.getYCenter() - k*S*sin(angle - da);
  }
  
  public void showLateralSensors() {
    noStroke();
    fill(255, 0, 0);
    ellipse(lateralSensors[0][0], lateralSensors[0][1], k*0.02, k*0.02);
    fill(0, 255, 255);
    ellipse(lateralSensors[1][0], lateralSensors[1][1], k*0.02, k*0.02);
  }
  
  public void showSensors() {
    noStroke();
    for(short i = 0; i < numSensors; i++) {
      fill(255*i/numSensors, 255 - 255*i/numSensors, 128);
      ellipse(coordSensors[i][0], coordSensors[i][1], k*Q/numSensors, k*Q/numSensors);
    }
  }
  
  public void showCenterAndWheels() {
    float angle = bot.getPosition();
    noStroke();
    fill(200, 90, 20);
    ellipse(bot.getXCenter(), bot.getYCenter(), k*0.02, k*0.02); // Centro
    fill(20, 90, 200);
    ellipse(bot.getXCenter() + k*raio*cos(angle + PI/2.0), bot.getYCenter() - k*raio*sin(angle + PI/2.0), k*0.02, k*0.02); // Roda esquerda
    fill(20, 190, 20);
    ellipse(bot.getXCenter() + k*raio*cos(angle - PI/2.0), bot.getYCenter() - k*raio*sin(angle - PI/2.0), k*0.02, k*0.02); // Roda direita
  }
  
  public void robot() {
      pushMatrix();
      translate(bot.getXCenter(), bot.getYCenter());
      rotate(PI/2.0 - bot.getPosition());
      
      strokeWeight(k*0.008/pRat);
      stroke(0);
      
      fill(80, 0, 0, 90);
      rect(-k*0.07/pRat, -k*0.18/pRat, k*0.14/pRat, k*0.04/pRat);
      
      fill(255, 255, 0);
      triangle(0, -k*0.18/pRat, -k*0.1/pRat,  0, k*0.1/pRat, 0);
      
      fill(0);
      ellipse( + 0.1*k/pRat, 0, k*0.04/pRat, 0.1*k/pRat);
      ellipse( - 0.1*k/pRat, 0, k*0.04/pRat, 0.1*k/pRat);
      
      popMatrix();
  }
  
  public void robotImage() {
    pushMatrix();
    translate(bot.getXCenter(), bot.getYCenter());
    rotate(4.71238898 - bot.getPosition());
    image(this.ratao, 0, k*0.033, k/(pRat), k/(pRat));
    popMatrix();
  }
  
  public void circulos() {
    bot.circulo();
    bot.posicaoRadians(bot.getPosition() - PI/21);
    bot.circulo();
    bot.posicaoRadians(bot.getPosition() + PI);
    bot.circulo();
    bot.posicaoRadians(bot.getPosition() - PI/2); 
  }
  
  public void menu() {
    fill(0, 255, 155);
    textSize(20);
    text("vEsq: " + nf(v2, 1, 3) + " m/s" + "  vDir: " + nf(v1, 1, 3) + " m/s" + "  w: " + nf(bot.getVelAngular(), 3, 2) + " º/s", 0.8*comp, comp);
    text("Velocidade: " + nf(vC, 1, 2) + " m/s", 0.8*comp, 1.3*comp);
  }
  
  public void alvo() {
    stroke(200, 0, 0);
    strokeWeight(2);
    line(width/2.0 - comp, height/2.0, width/2.0 + comp, height/2.0);
    line(width/2.0, height/2.0 - comp, width/2.0, height/2.0 + comp);
    stroke(0, 200, 0);
    line(width/2.0 - w, height/2.0 - h, width/2.0 + w, height/2.0 + h);
    line(width/2.0 - w, height/2.0 + h, width/2.0 + w, height/2.0 - h);
  }
  
  public void escala() {
    fill(50);
    noStroke();
    rect(width*0.08, height*0.92, k, -k/8.0);
  }
  
  public boolean deuBug() {
    frontSensors();
    lateralSensors();
    for(char i = 0; i < numSensors; i++) {
      if(coordSensors[i][0] >= width  || coordSensors[i][0] < 0) return true;
      if(coordSensors[i][1] >= height || coordSensors[i][1] < 0) return true;
    }
    if(lateralSensors[0][0] >= width  || lateralSensors[0][0] < 0) return true;
    if(lateralSensors[0][1] >= height || lateralSensors[0][1] < 0) return true;
    if(lateralSensors[1][0] >= width  || lateralSensors[1][0] < 0) return true;
    if(lateralSensors[1][1] >= height || lateralSensors[1][1] < 0) return true;
    return false;
  }
  
  public int position() {
    int avr = 0;
    int soma = 0, max = 0, min = 1000;
    boolean on_line = false;
    for(char i = 0; i < numSensors; i++) {
      sensors[i] = (int) mat[(int)coordSensors[i][0]][(int)coordSensors[i][1]];
      value[i] = sensors[i];
      if(!mode) value[i] = 255 - sensors[i];
      if(value[i] > max) max = value[i];
      if(value[i] < min) min = value[i];
      if(value[i] > 80)  on_line = true;
      sensors[i] = 255 - sensors[i];
    }
    if(max == 0) max = 1;
    if(min == 255) min = 254;
    if(max == min) {
      if(max == 255) min = 254;
      if(max == 0  ) max = 1;
      if(max == min) max = max + 1;
    }
    if(on_line) {
      for(char i = 0; i < numSensors; i++) {
        value[i] = (int) map(value[i], min, max, 0, 1000); 
        //(int) map(value[i], 0, 255, 0, 1000);
        avr  += 1000*i*value[i];
        soma += value[i];
      }
      if(soma != 0) return (int) kalman.updateEstimate(avr/soma);    //avr/soma;
      else return (int) kalman.updateEstimate((numSensors - 1)*500); //(numSensors - 1)*500;
    }
    else {
      if(last_proportional < 0) return (int) kalman.updateEstimate((numSensors - 1)*1000); //(numSensors - 1)*1000;
      else return (int) kalman.updateEstimate(0); //0;
    }
  }
  
  public int readLine() {
    return readLine(sensors, false);
  }
  
  public int readLine(int sensors[], boolean mode) {
    int avr = 0;
    int soma = 0, max = 0, min = 1000;
    boolean on_line = false;
    for(char i = 0; i < numSensors; i++) {
      sensors[i] = (int) mat[(int)coordSensors[i][0]][(int)coordSensors[i][1]];
      value[i] = sensors[i];
      if(!mode) value[i] = 255 - sensors[i];
      if(value[i] > max) max = value[i];
      if(value[i] < min) min = value[i];
      if(value[i] > 80)  on_line = true;
      sensors[i] = 255 - sensors[i];
    }
    if(max == 0) max = 1;
    if(min == 255) min = 254;
    if(max == min) {
      if(max == 255) min = 254;
      if(max == 0  ) max = 1;
      if(max == min) max = max + 1;
    }
    if(on_line) {
      for(char i = 0; i < numSensors; i++) {
        value[i] = (int) map(value[i], min, max, 0, 1000);
        //(int) map(value[i], 0, 255, 0, 1000);
        avr  += 1000*i*value[i];
        soma += value[i];
      }
      if(soma != 0) return (int) kalman.updateEstimate(avr/soma);    //avr/soma;
      else return (int) kalman.updateEstimate((numSensors - 1)*500); //(numSensors - 1)*500;
    }
    else {
      if(last_proportional < 0) return (int) kalman.updateEstimate((numSensors - 1)*1000); //(numSensors - 1)*1000;
      else return (int) kalman.updateEstimate(0); //0;
    }
  }
  
  // Controle
  
  private void speedControl() {
    switch(contEsq) {
      //case 42: println(timeMicros);
    }
  }
  
  public void detectaCruzamento() {
    if(((int) mat[(int)lateralSensors[0][0]][(int)lateralSensors[0][1]] >= linha) == mode && ((int) mat[(int)lateralSensors[1][0]][(int)lateralSensors[1][1]] >= linha) == mode) {
      cruzamento = true;
      //print("\n" + "Cruzamento" + "\n\n" );
    } else {
      cruzamento = false; 
    }
  }

  public void sensorEsquerdo() { // Função que verifica se sensor esquerdo passou sobre a faixa lateral esquerda, caso ocorra contEsq++
    lastEsq = flagEsq;
    
    if(((int) mat[(int)lateralSensors[0][0]][(int)lateralSensors[0][1]] >= linha) == mode) flagEsq = true;  // Se a leitura do sensor lateral for maior que 127 significa que o robô leu a faixa a lateral, logo a flagEsq recebe true
    else flagEsq = false;                                                                   // caso contrário recebe false
  
    if(flagEsq && !lastEsq) {
      if(contEsq < marksSize) {
        if(getDistPercorrida() - dist_esq > 0.02) {
          //dist_atual = marksDist[contEsq]; // Precisa-se melhorar
          dist_esq = getDistPercorrida();
        } else return;
      }
      //else print("Deve ter erro no sensor esquerdo, contador esquerdo: " + (contEsq + 1));
      contEsq++;
      //speedControl();
      timeEsq = millis();
      sobreSensorEsq = true;
      //println("Esq: " + contEsq + " dist: " + nf(dist_atual, 1, 2) + " dm: " + nf(marksDist[contEsq - 1], 1, 2));
      if(printLateralSensors) println("Esq: " + contEsq + "  Dir: " + contDir);
    }
  
    //if(!flagEsq && lastEsq) {
    //}
  }
  
  public void sensorDireito() { // Funcão análoga a função sensorEsquerdo()
    if(contDir >= contagemMaxima) {
      finalStop();
      return;
    }
    lastDir = flagDir;
    if(contDir >  0) dist_atual += get_dif_dist();
    if(contDir <= 0) first_right_time = timeMicros;
    if(((int) mat[(int)lateralSensors[1][0]][(int)lateralSensors[1][1]] >= linha) == mode) flagDir = true;
    else flagDir = false;
  
    if(flagDir && !lastDir)  {
      if(getDistPercorrida() - dist_dir > 0.02) dist_dir = getDistPercorrida();
      else return;
      if(contDir == contagemMaxima - 1) {
        dist_final = getDistPercorrida();
        final_right_time = getTimeMicros();
        //println(dist_final);
        finalStop();
      }
      sobreSensorDir = true;
      contDir++;
      if(printLateralSensors) println("Esq: " + contEsq + "  Dir: " + contDir);
    }
  
    //if(!flagDir && lastDir) {
    //}
  }
  
  public void printSensoresLaterais() {
    println("Esq: " + contEsq + ", dir: " + contDir);
  }
  
  private int erro() {
    int p = ceil(100*this.bot.variacaoPosition(this.bot.getPointPosition(), this.bot.getPosition()));
    if(abs(p) > 2000) {
      if(last_proportional >=0 ) p = 2000;
      else p = -2000;
    }
    this.erroAbsoluto += abs(p);
    this.erroAbsolutoQuadrado += p*p;
    return p;
  }
  
  private int erro(float anguloVarredura) {
    anguloVarredura = 4000f/anguloVarredura;
    int p = ceil(anguloVarredura*this.bot.variacaoPosition(this.bot.getPointPosition(), this.bot.getPosition()));
    
    //println("position: " + bot.getPosition() + " point: " + bot.getPointPosition());
    
    if(abs(p) > 2000) {
      if(last_proportional >= 0) p = 2000;
      else p = -2000;
    }
    this.erroAbsoluto += abs(p);
    this.erroAbsolutoQuadrado += p*p;
    return p;
  }
  
  public void joystick(char letra, int vel) {
    this.timeMicros += dT;
    switch(letra) {
      case '6': speedMotors(vel/2, vel);
                break;
      case '4': speedMotors(vel, vel/2);
                break;
      case '8': speedMotors(vel, vel);
                break;
      case '2': speedMotors(-vel, -vel);
                break;
      case '9': speedMotors(2*vel/3, vel);
                break;
      case '7': speedMotors(vel, 2*vel/3);
                break;
      case '1': speedMotors(0, -vel);
                break;
      case '3': speedMotors(-vel, 0);
                break;
      case '5': speedMotors(0, 0);
                break;
    }
    bot.velAngular(bot.getVelAngular());
    if(mousePressed) {
      if(mouseButton == RIGHT) {
        bot.setCenter(mouseX, mouseY);
      }
    }
  }
  
  private void speedMotors(float a, float b) {
    if(allow_noise) {
      a = a + 0.1*(0.5 - noise(timeMicros/60000 + aleatorio1 + 100));
      b = b + 0.1*(0.5 - noise(timeMicros/60000 + aleatorio2 + 300));
    }
    if(considerInertia) bot.setPWM(k_m1*a, k_m2*b);
    else calculo(b, a);   
  }
  
  private void speedMotors(float[] speeds) {
    if (speeds.length != 2) return;
    int v1 = (int) constrain(speeds[0], -4095.0, 4095.0);
    int v2 = (int) constrain(speeds[1], -4095.0, 4095.0);
    speedMotors(v1, v2);
  }
  
  private void speedMotors(float[] speeds, float min, float max) {
    if (speeds.length != 2) return;
    //speeds[0] = speeds[1] = -2;
    int v1 = (int) constrain(map(speeds[0], min, max, -4095, 4095), -4095.0, 4095.0);
    int v2 = (int) constrain(map(speeds[1], min, max, -4095, 4095), -4095.0, 4095.0);
    //if((millis() / 10) % 10 == 0) println(v1, v2);
    speedMotors(v1, v2);
  }
  
  public void segueLinhaControle4() {
    int proportional = erro(this.varredura);
    int derivative = proportional - last_proportional;
    erro = proportional;

    if((proportional <= 0 && last_proportional >= 0) || (proportional >= 0 && last_proportional <= 0)) {
      integral = 0;
      numOscilacoes++;
    }
    
    integral += proportional;

    int power_difference = ceil(proportional*Kp + integral*Ki + derivative*Kd);
    
    //print("Erro: " + proportional + "  Último erro: " + last_proportional + "  " + "Derivative: " + derivative +  "  Integral: " + integral + "  Correção: " + power_difference + "  P: " + proportional*Kp + "  I: " + integral*Ki + "  D:" + derivative*Kd + "\n");
    
    last_proportional = proportional;
    
    int a = this.vel + power_difference;
    int b = this.vel - power_difference;
    
    if(a > 4095) {
      a = 4095;
    } else if(a < -4095) {
      a = -4095;
    }

    if(b > 4095) {
      b = 4095;
    } else if(b < -4095) {
      b = -4095;
    }
    
    speedMotors(a, b);
}

public void segueLinhaControle3() {
    int proportional = (numSensors - 1)*500 - position();
    int derivative = proportional - last_proportional;
    erro = proportional;
    
    this.erroAbsoluto += abs(erro);
    this.erroAbsolutoQuadrado += erro*erro;

    if((proportional <= 0 && last_proportional >= 0) || (proportional >= 0 && last_proportional <= 0)) {
      integral = 0;
      numOscilacoes++;
    }
    
    integral += proportional;

    int power_difference = ceil(proportional*Kp + integral*Ki + derivative*Kd);
    
    //print("Erro: " + proportional + "  Último erro: " + last_proportional + "  " + "Derivative: " + derivative +  "  Integral: " + integral + "  Correção: " + power_difference + "  P: " + proportional*Kp + "  I: " + integral*Ki + "  D:" + derivative*Kd + "\n");
    
    last_proportional = proportional;
    
    int a = this.vel + power_difference;
    int b = this.vel - power_difference;
    
    if(a > 4095) {
      a = 4095;
    } else if(a < -4095) {
      a = -4095;
    }

    if(b > 4095) {
      b = 4095;
    } else if(b < -4095) {
      b = -4095;
    }
    
    speedMotors(a, b);
  }
  
  /*
  public void followLine(float vel, boolean omg) { // Controle do Exu
    float position = readLine(sensors, mode) - 2000 + d_error;
    float error = position * 0.001 * angle_max / 2.0;
    float derivative = (2.0 * tf - ts) / (2.0 * tf + ts) * last_derivative + (2.0 / (2.0 * tf + ts)) * (error - last_error);
    float w = 0;
  
    if (abs(error) <= angle_integrative) {
      integral += ts * (error + last_error) / 2.0;
      w = Ki * integral;
    }
    w += Kp * error + Kd * derivative;
    
    // Velocidade angular de referência
    if(omg) w += vel / radius;
    
    //println(radius);
  
    last_derivative = derivative;
    last_error = error;
  
    speedMotorLeft  = vel + w * distRodas / 4.0;
    speedMotorRight = vel - w * distRodas / 4.0;
    
    //float velMax = 3.2;
    //float aux = 0;
    
    //if (w > 0) {
    //  speedMotorLeft  = vel;
    //  speedMotorRight = vel - 2 * w * distRodas / 4.0;
    //}
    //if (w < 0) {
    //  speedMotorLeft  = vel + 2 * w * distRodas / 4.0;
    //  speedMotorRight = vel ;
    //}
  
    //if (speedMotorLeft > velMax) {
    //  aux = speedMotorLeft - velMax;
    //  speedMotorLeft = velMax;
    //  speedMotorRight = speedMotorRight - aux;
    //}
    //if (speedMotorRight > velMax) {
    //  aux = speedMotorRight - velMax;
    //  speedMotorRight = velMax;
    //  speedMotorLeft = speedMotorLeft - aux;
    //}
    
    if(!considerInertia) calculo(speedMotorLeft, speedMotorRight);
  }
  */
  
  public void followLine(float vel, boolean omg) { // Controle do Exu
    float position = readLine(sensors, mode) - 2000 + d_error;
    float error = position * 0.001 * angle_max / 2.0;
    Control c = (error > 0) ? malha_ext_esq : malha_ext_dir;
    float derivative = (2.0 * c.tf - c.ts) / (2.0 * c.tf + c.ts) * c.last_derivative + (2.0 / (2.0 * c.tf + c.ts)) * (error - c.lastError);
    float w = 0;
  
    if (abs(error) <= angle_integrative) {
      c.integral += c.ts * (error + c.lastError) / 2.0;
      w = c.Ki * c.integral;
    }
    w += c.Kp * error + c.Kd * derivative;
    
    // Velocidade angular de referência
    if(omg) w += vel / radius;
  
    c.last_derivative = derivative;
    c.lastError = error;
    
    float velMax = 3.2;
    float aux = 0;
    
    speedMotorLeft  = vel + w * distRodas / 4.0;
    speedMotorRight = vel - w * distRodas / 4.0;
    
    //if (w > 0) {
    //  speedMotorLeft  = vel;
    //  speedMotorRight = vel - 2 * w * distRodas / 4.0;
    //}
    //if (w < 0) {
    //  speedMotorLeft  = vel + 2 * w * distRodas / 4.0;
    //  speedMotorRight = vel ;
    //}
  
    if (speedMotorLeft > velMax) {
      aux = speedMotorLeft - velMax;
      speedMotorLeft = velMax;
      speedMotorRight = speedMotorRight - aux;
    }
    if (speedMotorRight > velMax) {
      aux = speedMotorRight - velMax;
      speedMotorRight = velMax;
      speedMotorLeft = speedMotorLeft - aux;
    }
    
    if(!considerInertia) calculo(speedMotorLeft, speedMotorRight);
  }
  
  // Controles de velocidade
  
  public float[] weelSpeed(float r) {
    float[] vel = new float[2];
    float v = getVelLinear();
    float w = getVelAngular();
    vel[0] = v - w*r;
    vel[1] = v + w*r;
    return vel;
  }
  
  public void speedEncoderControl(Control c1, Control c2) {
    speedEncoderControl(c1);
    speedEncoderControl(c2);
    speedMotors(PWMLeft, PWMRight);
  }
  
  public void speedEncoderControl(Control c1, Control c2, float speed1, float speed2) {
    c1.vel = speed1;
    c2.vel = speed2;
    speedEncoderControl(c1);
    speedEncoderControl(c2);
    speedMotors(PWMLeft, PWMRight);
  }
  
  void speedEncoderControl(Control c) { // Função que faz o controle do robô, a qual deve receber uma variável do tipo ControlEncoder na qual há os parâmetros de controle
    float nowSpeed = (c.type == 'L') ? weelSpeed(raio)[0] : weelSpeed(raio)[1];
    if (Si) { // Para converter m/s em Hz
      nowSpeed = nowSpeed / (PI * diametroRoda);
      c.vel    = c.vel    / (PI * diametroRoda);
    }
    float error            = c.vel - nowSpeed;      // Erro de velocidade em Hz
    float derivative       = error - c.lastError;   // A variável derivative recebe a variação do erro
    float power_difference = 0;
  
    derivative = 1000000 * derivative / dT;
    if (abs(error) < c.errorMin) {
      c.integral += error * dT / 1000000; // O tempo deve estar em segundos
      c.integral = constrain(c.integral, -c.rangeError, c.rangeError);
      power_difference = c.integral * c.Ki;
    }
    power_difference += error * c.Kp + derivative * c.Kd;
    if(c.type == 'L') PWMLeft = power_difference;
    else PWMRight = power_difference;
    c.lastError = error;
  }
  
  public void parar(float distMax) { // UPDATE IN 29/06/2020
    float fora = 0;
    int minSensor = 9999999;
    int maxSensor = 0;
    boolean sair = false;
    for (int i = 0; i < numSensors; i++) {
      if (sensors[i] > maxSensor) maxSensor = sensors[i];
      if (sensors[i] < minSensor) minSensor = sensors[i];
    }
    if (maxSensor - minSensor < 75) {
      if (somaErros > erroMaximo) sair = true;
      somaErros++;
      dist_rot += getVelAngular()*distRodas*dT/(4000000.0 * PI);
    }
    else {
      somaErros = 0;
      sair = false;
      last_dis = getDistPercorrida();
      dist_rot = 0;
    }
    if (sair) {
      fora = getDistPercorrida() - last_dis + dist_rot;
      fora += abs(dist_rot*0.3);
      if (fora > distMax) saiuDaLinha = true;
    } else last_dis = getDistPercorrida();
  }
  
  // Estratégia
  public void ignition() {
    float dist = getDistPercorrida();
    if(dist > 0.60) return;
    vel_linear = constrain(map(dist, 0.0, 0.60, initialSpeed, matrizEstrategica[0][1]), initialSpeed, matrizEstrategica[0][1]);
  }
  
  public void finalStop() { // Ao finalizar a estratégia, o robô deve parar a uma distância de no máximo 1 metro
    float dist = getDistPercorrida();
    if(dist > dist_final + 0.55) {
      saiuDaLinha = true;
      return;
    }
    vel_linear = map(dist, dist_final,  dist_final + 0.60, matrizEstrategica[matrixSize - 1][1], 0);
    //if((getTimeMicros() / 1000) % 20 == 0) println(vel_linear); //println(dist, dist_final);
  }
  
  public void odometry() {
    if(contDir == 0) {
      ignition();
      return;
    }
    if(contDir >= contagemMaxima) return;
    if(last_index == matrixSize - 2) return;
    float dist = dist_atual;
    int index = 0;
    for(int i = last_index; i < matrixSize; i++) {
      if(matrizEstrategica[i][0] > dist) break;
      index = i;
    }
    //if(index == matrixSize - 1 && last_index == index) return;
    last_index = index;
    if(index == matrixSize - 2) {
      
      //vel_linear = matrizEstrategica[index][1];
      //dist_final = getDistPercorrida();
      //finalStop();
    }
    else {
      //if(millis()%10 == 0) println(index + 1);
      vel_linear = map(dist, matrizEstrategica[index][0], matrizEstrategica[index + 1][0], matrizEstrategica[index][1], matrizEstrategica[index + 1][1]);
    }
  }
  
  public void omegaSpeed() {
    if(contDir <= 0) return;
    float dist = dist_atual;
    int index = 0;

    for(int i = last_index2; i < omegaMatrixSize; i++) {
      if(matrizOmega[i][0] > dist) break;
      index = i;
    }
    if(last_index2 == index) return;
    last_index2 = index;
    if(index == omegaMatrixSize - 1) {
      radius  =       matrizOmega[omegaMatrixSize - 1][1];
      d_error = (int) matrizOmega[omegaMatrixSize - 1][2];
    } else {
      radius  =       matrizOmega[index][1]; //map(dist, matrizOmega[index][0], matrizOmega[index + 1][0], matrizOmega[index][1], matrizOmega[index + 1][1]);
      d_error = (int) matrizOmega[index][2]; //map(dist, matrizOmega[index][0], matrizOmega[index + 1][0], matrizOmega[index][2], matrizOmega[index + 1][2]);
    }
    select_pid(radius);
  }
  
  // Redes neurais
  private void updateVetEntrance() {
    float[] vet = new float[vetEntrance.getNumLinhas()];
    
    //vet[0] = neuronOutput[0];
    //vet[1] = neuronOutput[1];
    //vet[2] = getDistPercorrida();
    //vet[3] = erro;
    
    //vet[0] = bot.getVelAngular();
    //vet[1] = bot.getVelLinear();
    //vet[2] = getDistPercorrida();
    //vet[3] = erro;
    
    for(int i = 0; i < numSensors; i++) vet[i] = sensors[i];
    vet[numSensors + 0] = mat[(int)this.lateralSensors[0][0]][(int)this.lateralSensors[0][1]];
    vet[numSensors + 1] = mat[(int)this.lateralSensors[1][0]][(int)this.lateralSensors[1][1]];
    vet[numSensors + 2] = getDistPercorrida();
    vet[numSensors + 3] = erro;
    vet[numSensors + 4] = bot.getVelAngular();
    vet[numSensors + 5] = bot.getVelLinear();
    
    /*vet[0] = erro;
    vet[1] = mat[(int)this.lateralSensors[0][0]][(int)this.lateralSensors[0][1]];
    vet[2] = mat[(int)this.lateralSensors[1][0]][(int)this.lateralSensors[1][1]];
    vet[3] = getDistPercorrida(); */
    vetEntrance.updateValues(vet);
  }
  
  public void followLineNeuralSensors() {
    updateVetEntrance();
    neuron.updateEntrance(vetEntrance);
    neuron.feedforward();
    neuronOutput = neuron.getOutputVet();
    //neuronOutput[1] *= -1; if((millis()/10)%3 == 0) println(neuronOutput[0], neuronOutput[1]);
    speedMotors(neuronOutput, -2, 2.0);
    //speedMotors(4095, 4095);
    erro = (numSensors - 1)*500 - position();
    erroAbsoluto += abs(erro);
    erroAbsolutoQuadrado += erro*erro;
    integral += erro;
    last_proportional = erro;
  }
  
  // Velocidade -> cinemática

  public void calculo(float vEsq, float vDir) {
    this.v1 = vDir;
    this.v2 = vEsq;
    this.vC = (v1+v2)/2.0;
    
    this.dX = vC*cos(bot.getPosition())*dT/1000000.0;
    this.dY = vC*sin(bot.getPosition())*dT/1000000.0;
    
    this.dif_dist = sqrt(dX*dX + dY*dY);
    if(vel_linear < 0) dif_dist *= -1;
    this.distPercorrida += dif_dist;
    
    this.xC = bot.getXCenter() + dX*k;
    this.yC = bot.getYCenter() - dY*k;
    
    this.bot.setCenter(xC, yC);
    
    float w = (v1-v2) / ( 2.0*raio );
    this.bot.velAngular(w);
    
  }
  
  public void atualiza() {
    stroke(30, 180, 140);
    strokeWeight(this.k/50.0);
    for(short i = 0; i < t-1; i++) {
      if(coord[i][0] != coord[i+1][0] || coord[i][1] != coord[i+1][1]) point(coord[i][0], coord[i][1]);
      coord[i][0] = coord[i+1][0];
      coord[i][1] = coord[i+1][1];
    }
    coord[t-1][0] = (short) bot.getXCenter();
    coord[t-1][1] = (short) bot.getYCenter();
    point(coord[t-1][0], coord[t-1][1]);
  }
  
  public void atualiza(color cor) {
    stroke(cor);
    strokeWeight(this.k/50.0);
    for(short i = 0; i < t-1; i++) {
      if(coord[i][0] != coord[i+1][0] || coord[i][1] != coord[i+1][1]) point(coord[i][0], coord[i][1]);
      coord[i][0] = coord[i+1][0];
      coord[i][1] = coord[i+1][1];
    }
    coord[t-1][0] = (short) bot.getXCenter();
    coord[t-1][1] = (short) bot.getYCenter();
    point(coord[t-1][0], coord[t-1][1]);
  }
  
  public void atualiza(int x, int y) {
    stroke(30, 30, 190);
    strokeWeight(this.k/50.0);
    for(short i = 0; i < t-1; i++) {
      if(coord[i][0] != coord[i+1][0] || coord[i][1] != coord[i+1][1]) point(coord[i][0], coord[i][1]);
      coord[i][0] = coord[i+1][0];
      coord[i][1] = coord[i+1][1];
    }
    coord[t-1][0] = (short) ceil(x);
    coord[t-1][1] = (short) ceil(y);
    point(coord[t-1][0], coord[t-1][1]);
  }
  
  public void atualiza(int x, int y, color cor) {
    stroke(cor);
    strokeWeight(this.k/50.0);
    for(short i = 0; i < t-1; i++) {
      if(coord[i][0] != coord[i+1][0] || coord[i][1] != coord[i+1][1]) point(coord[i][0], coord[i][1]);
      coord[i][0] = coord[i+1][0];
      coord[i][1] = coord[i+1][1];
    }
    coord[t-1][0] = (short) ceil(x);
    coord[t-1][1] = (short) ceil(y);
    point(coord[t-1][0], coord[t-1][1]);
  }
  
  public void direcao() {
    if(mouseButton == RIGHT) bot.setCenter(mouseX, mouseY);
    if((bot.getYCenter() >= 1.1*height) || (bot.getYCenter() <= -0.1*height) || (bot.getXCenter() >= 1.1*width) || (bot.getXCenter() <= -0.1*width)) bot.setCenter(random(0, width), random(0, height));
    b1.drawRectButton();
    b2.drawRectButton();
    if(!mousePressed) {
      b1.setVariacao(ceil(1000*noise(millis()/4000.0)));
      b2.setVariacao(ceil(1000*noise(10000.0 + millis()/4000.0)));
      calculo(b1.variacao*vel_max_robo/1000.0 - vel_max_robo*0.3, b2.variacao*vel_max_robo/1000.0 - vel_max_robo*0.3);
    } else {
      if(b1.rectIsPressed() || b2.rectIsPressed()) calculo(b1.variacao*vel_max_robo/1000.0 - vel_max_robo*0.3, b2.variacao*vel_max_robo/1000.0 - vel_max_robo*0.3);
      else calculo(map(mouseX, 0, width, -vel_max_robo, vel_max_robo), map(mouseY, 0, height, -vel_max_robo, vel_max_robo));
    }
  }
  
  public void control() {
    this.timeMicros += dT;
    segueLinhaControle4();
    bot.velAngular(bot.getVelAngular());
    if(mousePressed) {
      if(mouseButton == RIGHT) bot.setCenter(mouseX, mouseY);
      else {
        bot.setPointCenter(mouseX, mouseY);
      }
    }
    if(!som && abs(erro) < 1000  && dist(bot.getXPoint(), bot.getYPoint(), bot.getXCenter(), bot.getYCenter()) < raio*k) {
      time = millis();
      if(!mousePressed) bot.setPointCenter(random(width), random(height));
      som = true;
    }
    if(millis() - time >= 200) som = false;
    bot.ponto();
  }
  
  public void select_pid(float r) {
    if(controles.size() < 2) return;
    r = abs(r);
    int ind = 0;
    for(int i = 1; i < controles.size(); i++) {
      if(r < 0.15*i) {
        ind = i;
        break;
      }
    }
    Control ct = controles.get(ind);
    setPID(ct.Kp, ct.Ki, ct.Kd);
  }
  
  public void lineControl() {
    if(!deuBug()) {
      this.timeMicros += dT;
      sensorEsquerdo();
      sensorDireito();
      segueLinhaControle3();
      bot.velAngular(bot.getVelAngular());
      if(abs(integral) >= erroMaximo) saiuDaLinha = true;
    } else {
      saiuDaLinha = true;
      if(lastVel != 0) {
        setPID(0, 0, 0, 0);
        //println("lastKp: " + this.lastKp);
      }
    }
    if(millis() - timeEsq >= 50) sobreSensorEsq = false;
    if(mousePressed) {
      if(mouseButton == RIGHT) {
        bot.setCenter(mouseX, mouseY);
        setPID(lastKp, lastKi, lastKd, lastVel);
      }
    }
  }
  
  public void exuControl() {
    if(!deuBug()) {
      this.timeMicros += dT;
      sensorEsquerdo();
      sensorDireito();
      odometry();
      omegaSpeed();
      followLine(vel_linear, true);
      bot.velAngular(bot.getVelAngular());
      parar(0.2); // UPDATE saiuDaLinha in 29/04/2020
    } else {
      saiuDaLinha = true;
    }
    if(millis() - timeEsq >= 50) sobreSensorEsq = false;
    if(mousePressed) {
      if(mouseButton == RIGHT) {
        bot.setCenter(mouseX, mouseY);
      }
    }
  }
  
  public void exuControl(Control c1, Control c2) {
    if(!deuBug()) {
      this.timeMicros += dT;
      sensorEsquerdo();
      sensorDireito();
      odometry();
      omegaSpeed();
      
      vel_linear = 0.6;
      
      if(c1.log_file == null) {
        c1.log_file = new String[4];
        for(int i = 0; i < c1.log_file.length; i++) c1.log_file[i] = "";
      }
      else if(timeMicros % 1000 == 0 && c1.save_data) {
        c1.log_file[0] += concertaPonto(nf(getVelLinear(), 1, 3))       + "\n";
        c1.log_file[1] += concertaPonto(nf(vel_linear, 1, 3))           + "\n";
        c1.log_file[2] += concertaPonto(nf(getDistPercorrida(), 1, 3))  + "\n";
        c1.log_file[3] += concertaPonto(nf(timeMicros/1000000.0, 1, 3)) + "\n";
      }
      if(cont_control % fator == 0) followLine(vel_linear, true);
      if(considerInertia) {
        speedEncoderControl(c1, c2, speedMotorLeft, speedMotorRight);
        cont_control++;
      }
      
      //bot.velAngular(bot.getVelAngular());
      parar(0.2); // UPDATE saiuDaLinha in 30/06/2020
    } else {
      saiuDaLinha = true;
      //c1.log_file = "";
    }
    if(millis() - timeEsq >= 50) sobreSensorEsq = false;
    if(mousePressed) {
      if(mouseButton == RIGHT) {
        bot.setCenter(mouseX, mouseY);
      }
    }
  }
  
  public void degrau(Control c1, Control c2, float v1, float v2) {
    if(!deuBug()) {
      this.timeMicros += dT;
      
      if(c1.log_file == null) {
        c1.log_file = new String[4];
        for(int i = 0; i < c1.log_file.length; i++) c1.log_file[i] = "";
      }
      else if(timeMicros % 1000 == 0 && c1.save_data) {
        c1.log_file[0] += concertaPonto(nf(weelSpeed(raio)[0], 1, 3))   + "\n";
        c1.log_file[1] += concertaPonto(nf(weelSpeed(raio)[1], 1, 3))   + "\n";
        c1.log_file[2] += concertaPonto(nf(getDistPercorrida(), 1, 3))  + "\n";
        c1.log_file[3] += concertaPonto(nf(timeMicros/1000000.0, 1, 3)) + "\n";
      }
      
      if(considerInertia) {
        speedEncoderControl(c1, c2, v1, v2);
        cont_control++;
      }
      parar(0.2);
      //bot.velAngular(bot.getVelAngular());
    } else {
      saiuDaLinha = true;
      //c1.log_file = "";
    }
    if(millis() - timeEsq >= 50) sobreSensorEsq = false;
    if(mousePressed) {
      if(mouseButton == RIGHT) {
        bot.setCenter(mouseX, mouseY);
      }
    }
  }
  
  public void degrauPWM(Control c1, int m1, int m2) {
    if(!deuBug()) {
      this.timeMicros += dT;
      
      if(c1.log_file == null) {
        c1.log_file = new String[4];
        for(int i = 0; i < c1.log_file.length; i++) c1.log_file[i] = "";
      }
      else if(timeMicros % 1000 == 0 && c1.save_data) {
        c1.log_file[0] += concertaPonto(nf(weelSpeed(raio)[0], 1, 3))   + "\n";
        c1.log_file[1] += concertaPonto(nf(weelSpeed(raio)[1], 1, 3))   + "\n";
        c1.log_file[2] += concertaPonto(nf(getDistPercorrida(), 1, 3))  + "\n";
        c1.log_file[3] += concertaPonto(nf(timeMicros/1000000.0, 1, 3)) + "\n";
      }
      
      speedMotors(m1, m2);
      bot.velAngular(bot.getVelAngular());
    } else {
      saiuDaLinha = true;
      //c1.log_file = "";
    }
    if(millis() - timeEsq >= 50) sobreSensorEsq = false;
    if(mousePressed) {
      if(mouseButton == RIGHT) {
        bot.setCenter(mouseX, mouseY);
      }
    }
  }
  
  public void neuralControl() {
    if (!deuBug()) {
      this.timeMicros += dT;
      sensorEsquerdo();
      sensorDireito();
      
      //if(cont_control % fator == 0) 
      followLineNeuralSensors();
      //else speedMotors(neuronOutput, -2, 2.0);
      //cont_control++;
      
      parar(0.1);
    } else {
      this.saiuDaLinha = true;
      if (this.lastVel != 0) {
        setPID(0.0F, 0.0F, 0.0F, 0);
      }
    } 
    
    if (millis() - this.timeEsq >= 50L) this.sobreSensorEsq = false; 
    if (mousePressed && mouseButton == RIGHT) {
      this.bot.setCenter(mouseX, mouseY);
      setPID(this.lastKp, this.lastKi, this.lastKd, this.lastVel);
    } 
  }
  
  public void PIDText() {
    String[] str = loadStrings(recebeDados);
    String[] cond = loadStrings(atualiza);
    if(str.length == 4 && cond.length == 1){
      if(cond[0].equals("s")) {
        for(char i = 0; i < 4; i++) {
          values[i] = int(str[i]);
        }
        this.Kp = map(values[0], 0, amplitude, intervalo[0][0], intervalo[0][1]);
        this.Ki = map(values[1], 0, amplitude, intervalo[1][0], intervalo[1][1]);
        this.Kd = map(values[2], 0, amplitude, intervalo[2][0], intervalo[2][1]);
        this.vel = (int) map(values[3], 0, amplitude, intervalo[3][0], intervalo[3][1]);
      }
    }
  }
  
  public void salvarParametros() {
    this.lastXC = bot.getXCenter();
    this.lastYC = bot.getYCenter();
    this.lastAngle = bot.getPosition();
    this.last_kp = Kp;
    this.last_ki = Ki;
    this.last_kd = Kd;
  }
  
  public void reiniciarParametros() {
    bot.setCenter(lastXC, lastYC);
    bot.setPosition(lastAngle);
    bot.restart();
    bot.setDistPercorrida(0);
    distPercorrida = 0;
    saiuDaLinha = false;
    timeMicros = 0;
    contEsq = 0;
    contDir = 0;
    lastEsq = false;
    flagEsq = false;
    lastDir = false;
    flagDir = false;
    cruzamento = false;
    saiuDaLinha = false;
    last_proportional = 0;
    integral = 0;
    erroAbsoluto = 0;
    erroAbsolutoQuadrado = 0; 
    setPID(last_kp, last_ki, last_kd, 0);
    fitness = 0;
    last_error = 0;
    last_derivative = 0;
    vel_linear = initialSpeed;
    initialSpeed = vel_linear;
    dist_atual = 0;
    last_index = 0;
    last_index2 = 0;
    radius = 1000000;
    dif_dist = 0;
    dist_esq = 0;
    dist_dir = 0;
    dist_final = 0;
    dist_rot = 0;
    somaErros = 0;
    first_right_time = 0;
    final_right_time = 1000000000;
    cont_control = 0;
  }
  
   public void printPID(int[] values) {
      println("Kp: " + map(values[0], 0, amplitude, intervalo[0][0], intervalo[0][1]));
      println("Ki: " + map(values[1], 0, amplitude, intervalo[1][0], intervalo[1][1]));
      println("Kd: " + map(values[2], 0, amplitude, intervalo[2][0], intervalo[2][1]));
      println("Vel: " + (int) map(values[3], 0, amplitude, intervalo[3][0], intervalo[3][1]));
      print("\n------------------------------\n\n");
    }
    
    public void printNeuronEntrance() {
      println();
      for (int i = 0; i < neuron.layers[0].getNumLinhas(); i++) { 
        print(neuron.layers[0].mat[i][0], " " );
      }
    }
    
    public void printNeuronOutput() {
      println();
      for(int i = 0; i < neuron.layers[neuron.N_-1].getNumLinhas(); i++) { 
        print(neuron.layers[neuron.N_-1].mat[i][0], " " );
      }
    }
    
    public void printPID() {
      println("Kp: " + Kp);
      println("Ki: " + Ki);
      println("Kd: " + Kd);
      println("Vel: " + vel);
      print("\n------------------------------\n\n");
    }
    
    public void printFitness() {
      println("Fitness: " + nf(fitness*100.0, 1, 2) + " %");
    }
  
  // set and get
  
  Movimento get() {
    Movimento m = new Movimento(this.getXCenter(), this.getYCenter(), this.bot.getPosition(), this.k, this.considerInertia); // Erro grave em this.mode -> this.considerInertia
    m.setMatrix(mat);
    m.setVelMax(this.vel_max_robo);
    m.setPID(this.Kp, this.Ki, this.Kd, this.vel);
    m.setRecebeDados(recebeDados);
    m.setStartTime(this.startTime);
    m.setVariacaoTempo(this.dT);
    m.salvarParametros();
    m.setNeural_Network(neuron.get());
    m.vetEntrance = new Matrix(m.neuron.getEntranceLength(), 1);
    m.malha_ext_esq = this.malha_ext_esq.copy();
    m.malha_ext_dir = this.malha_ext_dir.copy();
    return m;
  }
  
  public int getAmplitude() {
    return this.amplitude;
  }
  
  public int getContEsq() {
    return this.contEsq;
  }
  
  public int getContDir() {
    return this.contDir;
  }
  
  public float get_dif_dist() {
    if(considerInertia) dif_dist = bot.get_dif_dist();
    return dif_dist;
  }
    
   public float getDistPercorrida() {
     if(considerInertia) this.distPercorrida = bot.getDistPercorrida();
     return this.distPercorrida;
   }
   
   public float getDx() {
     this.dX = bot.getDx();
     return this.dX;
   }
   
   public float getDy() {
     this.dY = bot.getDy();
     return this.dY;
   }
    
    public int getErro() {
      return this.erro;
    }
    
    public float getFitness() {
      return this.fitness;
    }
    
    public float getKd() {
      return this.Kd;
    }
    
    public float getKp() {
      return this.Kp;
    }
    
    public float getKi() {
      return this.Ki;
    }
    
    public float getMassa() {
       return this.bot.massa;
     }
     
     public float getMomentoDeInercia() {
       return this.bot.K;
     }
     
     public float getGravidade() {
       return this.bot.g;
     }
    
    public int getVel() {
      return this.vel;
    }
    
    public float getVelLinear() {
      if(!considerInertia) return vel_linear;
      return bot.getVelLinear();
    }
    
    public int getErroMaximo() {
      return this.erroMaximo;
    }
    
    public boolean getSobreSensorEsq() {
      return sobreSensorEsq;
    }
    
    public boolean getSobreSensorDir() {
      return sobreSensorDir;
    }
    
    public int[] getValues() {
     return this.values;
    }
    
    public float getXCenter() {
      return this.xC;
    }
    
    public float getYCenter() {
      return this.yC;
    }
    
    public boolean somStatus() {
      return this.som;
    }
    
    public void setCenter(float x, float y) {
      this.bot.setCenter(x, y);
    }
    
    public void setPosicao(float x, float y, float angle) {
      this.bot.setCenter(x, y);
      this.bot.setPosition(angle);
    }
    
    public long getErroAbsoluto() {
      return this.erroAbsoluto;
    }
    
    public long getErroAbsolutoQuadrado() {
      return this.erroAbsolutoQuadrado;
    }
    
    public long getErroTotal() {
      return this.integral;
    }
    
    public long getIntegral() {
      return this.integral;
    }
    
    public float getVelMax() {
      return this.vel_max_robo;
    }
    
    public float[][] getIntervalo() {
      return this.intervalo.clone();
    }
    
    public String[] getLeituraSensoresFrontais() {
      String[] leitura = new String[numSensors];
      for(char i = 0; i < numSensors; i++) leitura[i] = nf(sensors[i], 1, 0);
      return leitura;
    }
    
    public Neural_Network getNeural_Network() { 
      return this.neuron.get();
    }
    
    public String getPID() {
      String retorno = "";
      retorno += "Kp: " + map(values[0], 0, amplitude, intervalo[0][0], intervalo[0][1]) + "\n";
      retorno += "Ki: " + map(values[1], 0, amplitude, intervalo[1][0], intervalo[1][1]) + "\n";
      retorno += "Kd: " + map(values[2], 0, amplitude, intervalo[2][0], intervalo[2][1]) + "\n";
      retorno += "Vel: " + (int) map(values[3], 0, amplitude, intervalo[3][0], intervalo[3][1]) + "\n";
      //retorno += "\n------------------------------\n\n";
      return retorno;
    }
    
    public String getRobotPID() {
      String retorno = "Malha Externa\n";
      retorno += "Kp: " + Kp + "\n";
      retorno += "Ki:  " + Ki + "\n";
      retorno += "Kd: " + Kd + "\n";
      //retorno += "\n------------------------------\n\n";
      return retorno;
    }
    
    public boolean saiuDaLinha() {
      return this.saiuDaLinha;
    }
    
    public void setSaiuDaLinha(boolean s) {
      this.saiuDaLinha = s;
    }
    
    public int getStartTime() {
      return this.startTime;
    }
    
    public long getTimeMicros() {
      return this.timeMicros;
    }
    
    public long getTimeStart() {
      return this.first_right_time;
    }
    
    public long getTempoPercorrido() {
      long tempo = timeMicros - first_right_time;
      if(timeMicros > final_right_time) tempo = final_right_time - first_right_time;
      return tempo;
    }
    
    public int getVariacaoDeTempo() {
      return this.dT;
    }
    
    public float getVelAngular() {
     return bot.getVelAngular();
   }
   
   public void setAllowNoise(boolean allow) {
     this.allow_noise = allow;
   }
    
    public void setFitness(float ft) {
      this.fitness = ft;
    }
    
    public void setIntervalo(float[][] inter) {
      this.intervalo = inter;
    }
    
    public void setSobreSensorEsq(boolean s) {
      sobreSensorEsq = s;
    }
    
    public void setSobreSensorDir(boolean s) {
      sobreSensorDir = s;
    }
    
    public void setTimeMicros(long time) {
      this.timeMicros = time;
    }
    
    public void setVariacaoTempo(int t) { // dT em microssegundos
      this.dT = t;
      this.bot.setVariacaoTempo(t);
      this.ts = (dT / 1000000)*fator;
    }
    
    public void setMatrix(char matriz[][]) {
      this.mat = matriz;
    }
    
    public void setNeural_Network(int e, int o) {
      this.neuron = new Neural_Network(e, o);
      this.vetEntrance = new Matrix(this.neuron.getEntranceLength(), 1);
    }
    
    public void setNeural_Network(int e, int h, int o) {
      this.neuron = new Neural_Network(e, h, o);
      this.vetEntrance = new Matrix(this.neuron.getEntranceLength(), 1);
    }
    
    public void setNeural_Network(int[] vet) { 
      this.neuron = new Neural_Network(vet);
      this.vetEntrance = new Matrix(this.neuron.getEntranceLength(), 1);
    }
    
    public void setNeural_Network(int[] vet1, int[] vet2) {
      this.neuron = new Neural_Network(vet1, vet2);
      this.vetEntrance = new Matrix(this.neuron.getEntranceLength(), 1);
    }
    
    public void setNeural_Network(Neural_Network network) {
      this.neuron = network;
      this.vetEntrance = new Matrix(this.neuron.getEntranceLength(), 1);
    }
    
    public void setVelMax(float v) {
      this.vel_max_robo = v;
    }
    
    public void setPID(int p) {
      this.lastKp = this.Kp;
      this.lastKi = this.Ki;
      this.lastKd = this.Kd;
      this.lastVel = this.vel;
      this.Kp = 0;
      this.Ki = 0;
      this.Kd = 0;
      this.vel = p;
    }
    
    public void setProportional(float p) {
      this.lastKp = this.Kp;
      this.Kp = p;
    }
    
    public void setPD(float p, float d) {
      this.lastKp = this.Kp;
      this.lastKd = this.Kd;
      this.Kp = p;
      this.Kd = d;
    }
    
    public void setPI(float p, float i) {
      this.lastKp = this.Kp;
      this.lastKi = this.Ki;
      this.Kp = p;
      this.Ki = i;
    }
    
    public void setPID(float p, float i, float d) {
      this.lastKp = this.Kp;
      this.lastKi = this.Ki;
      this.lastKd = this.Kd;
      this.Kp = p;
      this.Ki = i;
      this.Kd = d;
    }
    
    public void setPID(float p, float i, float d, int v) {
      //println("Antes: " + lastKp);
      this.lastKp = this.Kp;
      //println("Depois: " + lastKp);
      this.lastKi = this.Ki;
      this.lastKd = this.Kd;
      this.lastVel = this.vel;
      this.Kp = p;
      this.Ki = i;
      this.Kd = d;
      this.vel = v;
    }
    
    public void setPID(int[] values) {
      if(values.length == 4) this.values = values;
      this.lastKp = this.Kp;
      this.lastKi = this.Ki;
      this.lastKd = this.Kd;
      this.lastVel = this.vel;
      this.Kp = map(values[0], 0, amplitude, intervalo[0][0], intervalo[0][1]);
      this.Ki = map(values[1], 0, amplitude, intervalo[1][0], intervalo[1][1]);
      this.Kd = map(values[2], 0, amplitude, intervalo[2][0], intervalo[2][1]);
      this.vel = (int) map(values[3], 0, amplitude, intervalo[3][0], intervalo[3][1]);
    }
    
    public void setStartTime(int t) {
      this.startTime = t;
    }
    
    public void setAnguloVarredura(float angle) {
      if(angle > 0 && angle <= 360) this.varredura = angle;
    }
    
    public void setRecebeDados(String local) {
      this.recebeDados = local;
    }
}
