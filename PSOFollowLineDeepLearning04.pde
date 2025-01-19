/* 21/12/20
 * circleArea()
 * Álgebra atualizado
 * Recurrent Neural Networks
 * Algoritmo térmico -> quase lá
 * PSO + Deep Learning
 * Dinâmica de motores cc
 * Dinâmica do robô
 */

import processing.sound.*;

SoundFile file;
SqrOsc sqr[];
Menu menu;
Movimento[] m = new Movimento[10];
Movimento n;
Button b1;
DNA dna;
Control leftSpeedPID, rightSpeedPID;

String envio [] = new String[1];
//String endereco = "data/enviarParaGrafico.txt";

boolean last         = false;
boolean lastEsq      = false;
boolean som          = false;
boolean showAtualiza = true;
boolean showMany     = true;
boolean showCircle   = false;
boolean interfacePID = false;
boolean control_continue = true;
PImage  pista;
char    pretoBranco [][];
char    pretoBranco2[][];
color   cor    = color(0, 255, 0);
int     ger    = 0;
int     indice = 0;
long    time;
int     sum_time = 3000;
float   velTempo = 1.0;

void setup() {
  size(1000, 500);

  pista = loadImage("Iron Cup 2019.png");
  float p = pista.height / (float) width;

  b1 = new Button(0.01*width, 0.02*height, 0.05*width, 0.05*width);

  pretoBranco  = new char[width][height];
  pretoBranco2 = new char[width][height];
  
  //sum_time = int(sum_time*velTempo);
  
  leftSpeedPID  = new Control(525, 1536, 0, 0, 'L');
  rightSpeedPID = new Control(525, 1536, 0, 0, 'R');
  
  leftSpeedPID.save_data = false;
  
  m[0] = new Movimento(240, 54, 0, 299, true);
  m[0].setAllowNoise(true);
  //m[0].setRecebeDados("");
  m[0].setStartTime(0);
  
  m[0].setPID(200, 0, 8);
  m[0].malha_ext_esq = new Control(200, 0, 8);
  m[0].malha_ext_dir = new Control(200, 0, 8);
  m[0].malha_ext_esq.tf = 1.824;
  m[0].malha_ext_dir.tf = 1.824;
  m[0].malha_ext_esq.ts = m[0].malha_ext_dir.ts = 0.001; // Tempo em segundos
  
  int[] deepLearning   = {11, 5, 2}; // Neurônios para a camada de entrada, camadas ocultas e camada de saída
  int[] temporalLayers = {2, 0, 0}; // Camadas Temporais, são usadas nas redes neurais recorrentes
  m[0].setNeural_Network(deepLearning, temporalLayers);
  
  menu = new Menu(width, 35);
  
  sqr = new SqrOsc[3];
  for (char i = 0; i < 3; i++) sqr[i] = new SqrOsc(this);

  pushMatrix();
  translate(width/2.0, height/2.0);
  rotate(1.5*PI);
  image(pista, 0, 0, height, height*p);
  popMatrix();

  escalaCinza();

  sqr[0].amp(1);
  sqr[0].freq(220.44);

  sqr[1].amp(1);
  sqr[1].freq(1500);

  m[0].setMatrix(pretoBranco);
  m[0].setVelMax(3.45);  // Em m/s
  m[0].setVariacaoTempo(1000); // Em microssegundos
  m[0].setSaiuDaLinha(true);
  m[0].salvarParametros();

  dna = new DNA(m[0].get(), 8000000, 15.8);
  
  n = m[0].get();
  for(char i = 1; i < m.length; i++) m[i] = m[0].get();
  
   //println("\nMovimento:\n");
   //for(int i = 0; i < m.length; i++) m[i].neuron.debuga();

  textSize(15);
  
  thread("trainNet");
  background(255);
  time = millis();
}

void draw() {
  //println(mouseY);
  imprime(pretoBranco);
  int num = int((((millis()*velTempo - m[0].getStartTime()) - m[0].getTimeMicros()/1000))*1000.0/m[0].getVariacaoDeTempo());
  if(m[0].saiuDaLinha() || m[0].getDistPercorrida() > 16.5 || m[0].getTimeMicros() > 15000000) { // dna.getGeracaoAtual() != ger || 
    //Control ct = leftSpeedPID;
    //if(ct.log_file != null && ct.save_data) {
    //  control_continue = false;
    //  //ct.saveLogFile();
    //  ct.saveDegrau();
    //}
    ger = dna.getGeracaoAtual();
    //m[0].reiniciarParametros();
    m[0] = n.get();
    m[0].neuron.updateValues(dna.getBestWeights(0).copy());
    m[0].setStartTime((int) ((millis() + 500)*velTempo));
    
    for (char i = 1; i < m.length; i++) {
      m[i] = n.get();
      //m[i].reiniciarParametros();
      m[i].neuron.updateValues(this.dna.getWeightsAndBias(i).copy());
    }
    
    randomColors();
  }
  info_robot(0);
  if(control_continue)
  for (int j = 0; j < num; j++) {
    for (char i = 0; i < m.length; i++) {
      m[i].neuralControl();
      //m[i].exuControl(leftSpeedPID, rightSpeedPID);
    }
  }
  somEsq();
  if (showAtualiza) m[0].atualiza(cor);
  if (keyPressed) {
    condSom();
    showAtualiza();
  }
  
  m[0].robotImage();
  if (showMany) for (int i = m.length - 1; i >= 1; i--) m[i].robotImage();
  
  //m[0].showSensors();
  //m[0].showLateralSensors();
  //m[0].showCenterAndWheels();
  
  if(showCircle) circleArea();
  
  //println(m[0].weelSpeed(1)[0]);
  
  menu.showMenu();
  //m[0].printNeuronOutput();
  //envio[0] = nf(m[0].getErro(), 1, 0);
  //saveStrings(endereco, envio);
  //saveStrings(endereco, m[0].getLeituraSensoresFrontais());
  time = millis();
}

void evolucao() {
  while(true) {
    dna.novaGeracao();
    dna.printGeracoesPID();
    dna.printFitness(0);
    //println("Vel: " + nf(dna.getVelMedia(0), 1, 2) + " m/s, dist: " + nf(dna.getDistPercorrida(0), 1, 2) + " m");
    //println("Tempo: " + nf(dna.getTempoGasto(0), 1, 2) + " s, erro: " + nf(dna.getErro(0), 1, 5));
    dna.printPID(dna.getMelhoresPID(0));
  }
}

public void trainNet() {
  while(true) {
    //dna.novaGeracaoNeural();
    dna.novaGeracaoPSO();
    //dna.printGeracoesRede();
    //dna.printFitness(0);
    dna.printFitnessBest();
  } 
}

void condSom() {
  if (key == 's' || key == 'S') som = true;
  else if (key == 'n' || key == 'N') som = false;
}

void showAtualiza() {
  if (key == 'a' || key == 'A') showAtualiza = true;
  if (key == 'd' || key == 'D') showAtualiza = false;
  if (key == 'm' || key == 'M') showMany = true;
  if (key == 'u' || key == 'U') showMany = false;
  if (key == 'i' || key == 'I') interfacePID = true;
  if (key == 'o' || key == 'O') interfacePID = true;
  if (key == 'c' || key == 'C') showCircle = true;
  else showCircle = false;
}

void showButton() {
  noStroke();
  b1.rectButton();
  if (b1.isItOn()) fill(200, 0, 0);
  else fill(200, 200, 0);
  rect(b1.getX(), b1.getY(), b1.getComprimento(), b1.getLargura());
}

void som() {
  if (m[0].somStatus() != last) {
    if (m[0].somStatus()) {
      sqr[0].play();
    } else {
      sqr[0].stop();
    }
  }
  last = m[0].somStatus();
}

void somSensorLateral() {
  while (true) {
    somEsq();
  }
}

void somEsq() {
  if (m[0].getSobreSensorEsq() != lastEsq) {
    if (m[0].getSobreSensorEsq()) {
      if (som) sqr[1].play();
    } else {
      sqr[1].stop();
    }
  }
  lastEsq = m[0].getSobreSensorEsq();
}

void escalaCinza() {
  loadPixels();
  int i, j;
  for (i = 0; i < height; i++) {
    for (j = 0; j < width; j++) {
      pretoBranco[j][i] = (char) ((red(pixels[i*width+j]) + green(pixels[i*width+j]) + blue(pixels[i*width+j]))/3);
      if(pretoBranco[j][i] < 55) pretoBranco[j][i] = 55;
      //else pretoBranco[j][i] = 0;
    }
  }
  updatePixels();
}

void imprime(char mat[][]) {
  loadPixels();
  int i, j;
  for (i = 0; i < height; i++) {
    for (j = 0; j < width; j++) {
      pixels[i*width+j] = color(mat[j][i]);
    }
  }
  updatePixels();
}

void imprimeInverso(char mat[][]) {
  loadPixels();
  int i, j;
  for (i = 0; i < height; i++) {
    for (j = 0; j < width; j++) {
      pixels[i*width+j] = color(255 - mat[j][i]);
    }
  }
  updatePixels();
}

void escalaCinza(int r) {
  if (r <= 0 || r > 100) r = 1;
  int i, j, x, y, cont, media;
  for (i = 0; i < height; i++) {
    //println("I: " + i);
    for (j = 0; j < width; j++) {
      media = 0;
      cont = 0;
      for (x = j-r; x < j+r; x++) {
        for (y = i - r; y < i+r; y++) {
          if (x >= 0 && x < width && y >= 0 && y < height) {
            if (sqrt((j-x)*(j-x) + (i-y)*(i-y)) <= r) {
              media += pretoBranco[x][y];
              cont++;
            }
          }
        }
      }
      if (cont != 0) media = media/cont;
      else media = pretoBranco[j][i];
      pretoBranco2[j][i] = (char) media;
    }
  }
}

void randomColors() {
  switch((int) random(3)) {
  case 0: 
    cor = color(90, 255, 90);
    break;
  case 1: 
    cor = color(255, 90, 90);
    break;
  case 2: 
    cor = color(0, 140, 255);
    break;
  }
}

void circleArea() {
  fill(0, 0, 200, 110);
  noStroke();
  ellipse(mouseX, mouseY, 50, 50);
}

String concertaPonto(String p) {
  char[] v = p.toCharArray();
  for(int i = 0; i < v.length; i++) {
    if(v[i] == ',') {
      v[i] = '.';
      break;
    }
  }
  return new String(v);
}

void info_robot(int select) {
  if(select >= m.length || select < 0) {
    println("Robô não existente");
    return;
  }
  fill(255);
  textSize(14);
  text("V (difer.):    " + nf(m[select].getVelLinear() - m[select].vel_linear, 1, 2) + " m/s\n" +
       "V (exigida): "   + nf(m[select].vel_linear, 1, 2)                            + " m/s\n" +
       "V (robô):     "  + nf(m[select].getVelLinear(), 1, 2)                        + " m/s\n" +
       "T: " + nf(m[select].getTempoPercorrido()/1000000.0, 2, 3) + " s\n" + //
       //"T: " + nf(m[select].getTimeMicros()/1000000.0, 2, 2) + " s\n" +
       "D: " + nf(m[select].getDistPercorrida(), 2, 2)       + " m", 170, 82);
  if(velTempo != 1.0) {
    textSize(30);
    text(nf(velTempo, 1, 2) + " X", 700, 100);
  }
}
