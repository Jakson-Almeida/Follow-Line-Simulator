public class Aba {
  private float xAba;
  private float yAba;
  private float comprimento;
  private float largura;
  private float compText = 100;
  private color corAba    =   color(200);
  private color corTextBox =  color(80, 80, 150, 220);
  private color corAbaSobre = color(255);
  private boolean cond      = false;
  private boolean estaSobre = false;
  private boolean estaSobreCampoTexto = false;
  private String nome  = "";
  private String text  = null;
  private String text2 = null;
  private Button btnAba, btnCampText;
  private BarraLateral[] lateralBar = null;
  
  Aba(float x, float y, float c, float l) {
    this.xAba = x;
    this.yAba = y;
    this.comprimento = c;
    this.largura = l;
    btnAba = new Button(x, y - l, c, l);
    btnCampText = new Button(x, y, c, compText);
  }
  
  public void showAba() {
    noStroke();
    estaSobre = btnAba.rectIsPressed();
    if(!estaSobre && !cond) fill(corAba);
    else fill(corAbaSobre);
    arc(xAba + largura/4.0, yAba, largura*0.5, 2.0*largura, PI, 1.5*PI);
    rect(xAba + largura/4.0 - 1, yAba - largura, comprimento - largura/2.0 + 2, largura);
    arc(xAba + comprimento - largura*0.25, yAba, largura*0.5, 2.0*largura, 1.5*PI, 2.0*PI);
    fill(0);
    textSize(14);
    text(nome, xAba + largura/2.0, yAba - largura*0.3);
  }
  
  public void textBox() {
    fill(corTextBox);
    noStroke();
    rect(xAba, yAba, comprimento, compText);
    fill(255);
    textSize(12);
    try {
      text(text,  xAba + 0.1*comprimento, yAba + comprimento * 0.15);
      text(text2, xAba + 0.8*comprimento, yAba + comprimento * 0.15);
      for(char i = 0; i < lateralBar.length; i++) lateralBar[i].showBarraLateral();
    } catch(NullPointerException exception) {
    }
  }
  
  public void setCompText(float comp) {
    btnCampText = new Button(xAba, yAba, comprimento, comp);
    this.compText = comp;
  }
  
  public void setCorAba(color cor) {
    this.corAba = cor;
  }
  
  public void setCond(boolean cond) {
    this.cond = cond;
  }
  
  public void setLateralBar(float[] vetY) {
    this.lateralBar = new BarraLateral[vetY.length];
    for(char i = 0; i < vetY.length; i++) lateralBar[i] = new BarraLateral(xAba + 0.1*comprimento, yAba + vetY[i], comprimento*0.8, 15);
  }
  
  public void setNome(String nome) {
    this.nome = nome;
  }
  
  public void setTextBox(String text) {
    this.text = text;
  }
  
  public void setTextBox2(String text) {
    this.text2 = text;
  }
  
  public color getCorAba() {
    return this.corAba;
  }
  
  public boolean getEstaSobre() {
    return this.estaSobre;
  }
  
  public boolean getEstaSobreCampoTexto() {
    this.estaSobreCampoTexto = btnCampText.rectIsPressed();
    return this.estaSobreCampoTexto;
  }
  
  public float getVariacao(int i) {
    return lateralBar[i].getVariacao();
  }
}

public class Menu {
  private float comprimento;
  private float largura;
  private char  ind          = 0;
  private float opacity      = 80.0;
  private float[] barrasAba1 = {95, 135, 175, 215};
  private float[] barrasAba2 = {55};
  private boolean lastAba   = false;
  private boolean menuCond  = false;
  private boolean lastCampText = false;
  private color  corFundo  = color(255);
  private Button btnMenu;
  private Button btnSettings;
  private Aba[] abas = new Aba[4];
  private String[] texts = new String[abas.length];
  
  Menu(float c, float l) {
    this.comprimento = c;
    this.largura = l;
    this.btnMenu = new Button(0, 0, c, l);
    this.btnSettings = new Button(0, 0, l, l);
    for(char i = 0; i < abas.length; i++) abas[i] = new Aba(60 + i*230.0, l, 210.0, l*0.8);
    abas[0].setNome("Algoritmo Genético");
    abas[1].setNome("Física");
    abas[2].setNome("Controle PID");
    abas[3].setNome("Neural Network");
    //abas[1].setTextBox2(nf(m[0].getMassa(), 1, 2) + " kg\n" + nf(m[0].getMomentoDeInercia(), 1, 3) + " kg*m²\n" + nf(m[0].getGravidade(), 1, 2) + " m/s²");
    abas[0].setCompText(240);
    abas[2].setCompText(120);
    abas[3].setCompText(110);
    texts[0] = "Geração atual:\nTamanho da população\nFitness:\nMutacao:\n\nPeso estabilidade:\n\nPeso velocidade:\n\nPeso distância:";
    texts[1] = "Massa \nMomento de inércia \nGravidade";
    texts[3] = "Entrada: " + m[0].neuron.getEntranceLength() + "\nOculta: " + m[0].neuron.getHiddenLength() + " camadas" + "\nSaida: " + m[0].neuron.getOutputLength();
    abas[0].setLateralBar(barrasAba1);
    abas[1].setLateralBar(barrasAba2);
    for(char i = 0; i < abas.length; i++) abas[i].setTextBox(texts[i]);
  }
  
  public void showMenu() {
    if(lastAba && abas[ind].getEstaSobreCampoTexto()) {
      abas[ind].setCond(true);
      menuCond = true;
      lastAba = false;
    }
    if(btnMenu.rectIsPressed() || menuCond) {
      noStroke();
      fill(corFundo, opacity);
      rect(0, 0, comprimento, largura);
      stroke(0);
      strokeWeight(1);
      if(btnSettings.rectIsPressed()) fill(230);
      rect(0, 0, largura, largura);
      for(char i = 1; i < 4; i++) line(largura*0.2, largura*0.25*i, largura*0.8, largura*0.25*i);
      for(char i = 0; i < abas.length; i++) {
        abas[i].showAba();
        if(abas[i].getEstaSobre()) {
          abas[i].textBox();
          lastAba = true;
          ind = i;
        }
      }
      if(lastAba || menuCond) {
        if(ind == 0) {
          abas[0].setTextBox2(nf(dna.getGeracaoAtual()     , 1, 0) + "\n" +
                              nf(dna.getTamanlhoPopulacao(), 1, 0) + "\n" + 
                              nf(100.0*dna.getFitness(0)   , 2, 1) + " %\n" + 
                              nf(100.0*dna.getMutacao()    , 2, 1) + " %\n\n" + 
                              nf(dna.pesoEstabilidade      , 1, 2) + " \n\n" +
                              nf(dna.pesoVel               , 1, 2) + " \n\n" +
                              nf(dna.pesoDist              , 1, 2));
          if(mousePressed) {
            dna.setMutacao(map(abas[0].getVariacao(0), 0, 1000, 0, 1.0));
            dna.pesoEstabilidade = map(abas[0].getVariacao(1), 0, 1000, 0, 1.0);
            dna.pesoVel          = map(abas[0].getVariacao(2), 0, 1000, 0, 1.0);
            dna.pesoDist         = map(abas[0].getVariacao(3), 0, 1000, 0, 1.0);
          }
        }
        if(ind == 2) abas[2].setTextBox(m[0].getRobotPID());
      }
      if(menuCond) abas[ind].textBox();
    }
    if(menuCond) {
      if(!abas[ind].getEstaSobreCampoTexto()) {
        menuCond = false;
        abas[ind].setCond(false);
      }
      lastCampText = abas[ind].getEstaSobreCampoTexto();
    }
  }
}

public class BarraLateral {
  private color corBtn   = color(200);
  private Button btnBarra;
  
  BarraLateral(float x, float y, float c, float l) {
    this.btnBarra = new Button(x, y, c, l);
    this.btnBarra.setCor(corBtn);
  }
  
  public void showBarraLateral() {
    this.btnBarra.drawRectButton();
  }
  
  public float getVariacao() {
    return btnBarra.getVariacao();
  }
  
}
