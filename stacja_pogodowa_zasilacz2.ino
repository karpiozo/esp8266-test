#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_ADS1015.h>
#include <Arduino.h>
#include <Time.h>
#include <TimeLib.h>
#include <Streaming.h>

#define DHTPIN 12
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

Adafruit_ADS1015 ads1015; 

void rpm1 ();
void rpm0 ();
void wysylanie_danych_na_serwer ();
void wind2 ();
void procedura_wyslij ();
String getTime();
int ktory_miesiac(String miesiac_);
String odpowiedz();
void data();
void ustaw_godzine();
void policz_kier();

const char* ssid              = "telebit";
const char* password          = "need4speed";
const char* mac_address       = "33:33:33:33:33:55";
unsigned int t_wyslij         = 10;               //czas w sekundach  (wysyła dane co x sekund)

byte ane = 0;
byte error = 0;
int  kier = 0;      //wartosc w stopniach 
float kier_sr = 0;  //srednia wartosc w stopniach
float kierunek = 0;   //wartosc analogowa
float kier_min = 0;
float kier_max = 0;
int pomiar=0;
int ki = 0;
int liczba_krokow_usredniania = 20;
int rok = 0;
byte miesiac = 0;
byte dzien = 0;
byte godzina = 0;
byte minuta = 0;
byte sekunda =0;
unsigned long time0 = millis();     //polaczenie z WIFI //setup
unsigned long time1 = millis();     //odczyt kierunku   //loop
unsigned long time2 = millis();     //wysyłanie danych  //loop
unsigned long time3 = millis();     //synchro zegara    //loop
unsigned long time4 = millis();     //funkcja getTime i odpowiedz
unsigned long time5 = millis();     //czujnik DHT22     //loop
unsigned long time6 = millis();     //restart           //loop
volatile unsigned long licznik = 0;
volatile unsigned long time00 = 0;
volatile unsigned long time01 = 0;
volatile unsigned long delta = 0;
volatile float pred_wiatr = 0;
volatile float pred_wiatr_moc = 0;
volatile float pred_max = 0;
volatile float V = 0;
unsigned long bouncetime = 0;
unsigned int t_polacz         = 20;              //czas w sekundach  (max czas oczekiwania na połaczenie wifi)
unsigned int t_kierunek       = 500;               //czas w milisekundach  (kierunek mierzy co x milisekund)
float predkosc_wiatru = 0;
float predkosc_wiatru_moc = 0;
float temperatura = 0;
float wilgotnosc = 0;
float napiecie_bat = 0;
float kx = 0;
float ky = 0;
const float pi=3.14159;

void setup() {
  Serial.begin(115200);     //dekalaracja komunikacji z modułem, w nawiasie predkosc transmisji w bodach, standardowo 115200,
  delay(2000);
  Serial.println("");
  Wire.begin(5,4);          //ustawienie magistrali I2C na pinach SDA - 5, SCL - 4
  ads1015.begin();          // inicjalizacja przetwornika ADC
  ads1015.setGain(GAIN_TWO);      //rozdzielczosc czujnika ADC
  //delay(1000);
  pinMode(0,INPUT_PULLUP);        //deklaracje pinów

  pinMode(13, INPUT_PULLUP);      //DAVIS
  pinMode(14, INPUT_PULLUP);      //BIALY
  pinMode(16, INPUT_PULLDOWN_16);      //Jeśli stan niski to podąłczony czujnik biały

  dht.begin();            //inicjalizacja czujnika DHT22 mierzacego temp i wilgotosc
  yield();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);                     //laczymy sie z WIFI zeby wyslac dane
  yield();
  time0=millis();
  while (WiFi.status() != WL_CONNECTED)
    {
      yield();
      Serial.print(".");                    //czeka na polaczenie
      if((millis()-time0)>(t_polacz*1000))  //po przekroczeniu zadanego czasu warunek prawdziwy
        {
          Serial.println("Problem z WIFI");
          ESP.restart();                    //restart modemu
          delay(1000);
        }
      delay(500);
    }
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  if(digitalRead(16)==HIGH)   //sprawdza jaki anemometr, stan niski - biały, wysoki - Davis
    {
      ane=1;                    //DAVIS
      Serial.println("Podlaczono anemometr DAVIS");
    }
  else
    {
      ane=0;                    //BIALY
      Serial.println("Podlaczono anemometr BIALY");
    }
    
  data();               //pobiera godzine z internetu
  ustaw_godzine();      //synchronizuje zegar wew z pobranym czasem

  if(ane==1)
    {
      attachInterrupt(digitalPinToInterrupt(13), rpm1, RISING);    //deklaracja przerwan do pomiaru predkosci
    }
  else
    {
      attachInterrupt(digitalPinToInterrupt(14), rpm0, RISING);
    }

  time1 = millis();
  time2 = time1;
  time3 = time1;
  Serial.print("Zaczynamy ");
  Serial.println(time2);
  time00 = millis();
  time01 = time00;
}


void loop() 
{
  pomiar=0;
  if(ane==0)
  {
    pomiar = ads1015.readADC_SingleEnded(0);        //odczyt wartosci analogowej
    yield();
    //Serial.print("Pomiar kierunku: ");
    //Serial.println(pomiar);
//srednia ciagniona
    kierunek = kierunek * liczba_krokow_usredniania;
    kierunek += pomiar;
    kierunek = kierunek / (liczba_krokow_usredniania + 1);
    if(pomiar > kier_max)
      kier_max=kierunek;
    if(pomiar < kier_min)
      kier_min=kierunek;
  }
  
   if((millis()-time1)>=t_kierunek)   //wykonuje sie raz na t_kierunek zadany na poczatku
  {  
    if(ane==1)
    {
      kierunek = ads1015.readADC_SingleEnded(2);    //pomiar kierunku wiatru (DAVIS)
      yield();
    }
    time1 = millis();
    wind2();             //zmiana wartosci analogowej na kierunek w stopniach (WH)
    ki++;
  }
  
   if((millis()-time2)>=(t_wyslij*1000))  //wykonuje sie raz na t_wyslij zadany na poczatku
  {  
    //liczy predkosc 
    time2=millis();
    if(ane==1)
      {
        if(licznik==0)              //w przypadku braku wiatru zabezpiecza przed dzieleniem przez 0 w kolejnym kroku
        {
          predkosc_wiatru=0;
          predkosc_wiatru_moc=0;
        }
        else
        {
          predkosc_wiatru = pred_wiatr/float(licznik);              //obliczamy srednia arytmetyczna predkosci wiatru
          predkosc_wiatru_moc = pow(pred_wiatr_moc/float(licznik),1.0/3.0);    //obliczamy srednia predkosc wiatru skorelowana z moca
        }
 //       predkosc_wiatru=2.25/(float(delta)/1000);
 //       predkosc_wiatru=float(licznik)*2.25/float(t_wyslij); //przeliczenie na mph, 2.25 z noty katalogowej
      }
    else
      {
        if(licznik==0)
        {
          predkosc_wiatru=0;
          predkosc_wiatru_moc=0;
        }
        else
        {
          predkosc_wiatru = pred_wiatr/float(licznik);
          predkosc_wiatru_moc = pow(pred_wiatr_moc/float(licznik),1.0/3.0);
        }
 //       predkosc_wiatru = float(licznik)/float(t_wyslij)*1.492;  //przeliczenie na mph, 1.492 z noty katalogowej
      }
    pred_wiatr_moc=0;   //zerujemy zmienne pomocnicze do obliczeń w kolejnym przedziale czasu
    pred_wiatr=0;
    licznik=0;
    
    pomiar = ads1015.readADC_SingleEnded(1);      //pomiar wartości analogowej napiecia na baterii
    yield();
    Serial.print("Napiecie na beterii wynosi: ");
    napiecie_bat=float(pomiar)/122.6;             //przelicza wartosc analogowa na rzeczywisa wartosc napiecia
    Serial.print(napiecie_bat);              //można wykalibrować mierząc napiecie miernikiem i porownoując je z podanym przez program
                                                    //napięcie baterii w mV dzielimy przez napięcie na zacisku A1 w odniesieniu do masy podane w mV
    Serial.println("V");
    
    temperatura = dht.readTemperature(true);      //odczyt temp, teraz jest w F jesli chcemy w C to zostawic pusty nawias()
    yield();
    wilgotnosc = dht.readHumidity();              //odczyt wilgotnosci
    yield();
    time5=millis(); 
                          //w przypadku nieprawidlowego pomiaru probuje go powtorzyc
    while( !(temperatura < 125  && temperatura > 0) && millis()<(time5+2000))
    {
      temperatura = dht.readTemperature(true);
      yield();
    }
    time5=millis();
    while( !(wilgotnosc < 100 && wilgotnosc > 0) && millis()<(time5+2000))
    {
      wilgotnosc = dht.readHumidity();
      yield();
    }

    policz_kier();      //oblicza usredniony kierunek wiatru
    kx=0;               //zeruje zmienne pomocnicze
    ky=0;
    ki=0;
    
    Serial.print("Predkosc wiatru: ");
    Serial.print(predkosc_wiatru);
    Serial.print(",   pred_moc: ");
    Serial.println(predkosc_wiatru_moc);
    Serial.print("Kierunek:  ");
    Serial.println(kier_sr);
    Serial.print("Temperatura: ");
    Serial.print(temperatura);
    Serial.print("    Wilgotnosc: ");
    Serial.println(wilgotnosc);
    Serial.print(millis());
    Serial.println();

    wysylanie_danych_na_serwer ();    //wysyla dane na serwer
    kier_sr=0;                        //zeruje zmienne pomocnicze
    pred_max=0;
    if(error==1)      //jesli wystapi problem z wifi lub serwerem i dane nie zostana wyslane to prawda
    {
      Serial.println("Problemy z wyslaniem danych - RESTART");
      ESP.restart();      //restart modemu
      delay(1000);
    }
    
  if(digitalRead(16)==HIGH)   //sprawdza jaki anemometr, stan niski - biały, wysoki - Davis
    {
      ane=1;                    //DAVIS
    }
  else
    {
      ane=0;                    //BIALY
    }
  }
  
  if((millis()-time3)>=60000*60*12)    //synchronizacja zegara co 12h
  {
    time3 = millis();
    data();
    ustaw_godzine();
  }
  
  if(hour()==0 && minute()==0 && second()<10)    //restart o północy
  {
    ESP.restart();
    delay(1000);
  }
}

void rpm1 ()   //wywoluje sie przez przerwanie, tj sygnał z anemometru
{
  if((millis()-bouncetime)>20)        //zabezpiecza przed wywołaniem funkcji poprzez drganie styków
  {
    bouncetime = millis();            //odczytuje czas w milisekundach
    time01=bouncetime;                //podstawia pod zmienna time01 obecny czas
    delta=time01-time00;              //oblicza czas pomiedzy obecnym (time01) i porzednim sygnałem (time00)
    time00=time01;                    //nadpisuje czas obecny do zmiennej time00
    V=2.25/(float(delta)/1000);       //oblicza predkość wiatru dzieląc wartość prędkości odpowiadająca jednemu obrotowi na sekunde przez czas potrzebny na jeden obrót
    pred_wiatr += V;                  //sumuje obliczone predkosci
    pred_wiatr_moc += V*V*V;          //sumuje trzecie potegi predkosci
    licznik ++;                       //zlicza obroty od ostatniego wyslania danych na serwer
    if(V>pred_max)
      pred_max=V;                     //zapisuje najwieksza predkosc jaka wystapila w danym przedziale czasu
  }
}

void rpm0 ()   //wywoluje sie przez przerwanie (odczytano sygnał z anemometru)
{
  if((millis()-bouncetime)>20)        //zabezpiecza przed wywołaniem funkcji poprzez drganie styków
  {
    bouncetime = millis();            //odczytuje czas w milisekundach
    time01=bouncetime;                //podstawia pod zmienna time01 obecny czas
    delta=time01-time00;              //oblicza czas pomiedzy obecnym (time01) i porzednim sygnałem (time00)
    time00=time01;                    //nadpisuje czas obecny do zmiennej time00
    V=1.492/(float(delta)/1000);      //oblicza predkość wiatru dzieląc wartość prędkości odpowiadająca 
//                                      jednemu obrotowi na sekunde przez czas potrzebny na jeden obrót
    pred_wiatr += V;                  //sumuje obliczone predkosci
    pred_wiatr_moc += V*V*V;          //sumuje trzecie potegi predkosci
    licznik ++;                       //zlicza obroty od ostatniego wyslania danych na serwer
    if(V>pred_max)
      pred_max=V;                     //zapisuje najwieksza predkosc jaka wystapila w danym przedziale czasu
  }
}

void wysylanie_danych_na_serwer ()
{
  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  yield();
  const int httpPort = 80;
  if (!client.connect("widmo.tele.biz", httpPort)) {    //sprawdza polaczeie z serwerem
    yield();
    Serial.println("connection not OK");
    error=1;
    return;
    yield();
  }
 // delay(1000)

   String getget = "";    //skejamy get'a do wyslania
   getget += "GET /AutomaticDeviceData/createDeviceData?device_ID=";
   getget += mac_address;
   getget += "&dateutc=";
   getget += year();
   if(month()<10)
    getget += "-0";
   else
    getget += "-";
   getget += month();
   if(day()<10)
    getget += "-0";
   else
    getget += "-";
   getget += day();
   getget += "+";
   if(hour()<10)
    getget += "0";
   getget += hour();
   getget += "%3A";
   if(minute()<10)
    getget += "0";
   getget += minute();
   getget += "%3A";
   if(second()<10)
    getget += "0";
   getget += second();
   getget += "&winddir=";
   getget += kier_sr;
   getget += "&windspeedmph=";
   getget += predkosc_wiatru;
   getget += "&windgustmph=";
   getget += predkosc_wiatru_moc;
   getget += "&humidity=";
   getget += wilgotnosc;
   getget += "&tempf=";
   getget += temperatura;
   getget += "&rainin=0.000&dailyrainin=0.000&baromin=30.15019&dewptf=0.0&batterylevel=";
   getget += napiecie_bat;
   getget += " HTTP/1.1\r\n";

   Serial.println();
   Serial.println(getget);
   // Make a HTTP request:
    client.print(getget);
    client.print("Host: widmo.tele.biz\r\n");
    client.print("Connection: close\r\n\r\n"); 
    yield();
    String odp = odpowiedz(); //sprawdza czy odpowiedz serwera jest OK
    if (odp!="OK")
    {
      error=1;
      return;
      yield();
    }
    Serial.println(odp);
    client.stop();
    yield();
}

void wind2 ()
{
  if(ane==1) //DAVIS
    {
      kier=map(kierunek,0,1636,0,359);    //przeliczana jest wartosc analogowa na kierunek wiatru
      kx += V*sin(kier*PI/180);           //obliczenie skladowej x oraz sumowanie jej
      ky += V*cos(kier*PI/180);           //obliczenie skladowej y oraz sumowanie jej
    }
  else  //WH
    {
      kierunek=pomiar;
      //Serial.print("Kierunek: ");
      if(kier_min<=18 && kier_max>=75 && kier_max<=125)   
        {                  //wykonuje sie gdy kierunek oscyluje pomiedzy 45 a 90 st
          if(kierunek<=50)
            kier=90;
          else
            kier=45;
          return;
          yield();
        }
      if(kier_max>=950 && kier_min>=150 && kier_min<=220)
        {                 //wykonuje sie gdy kierunek oscyluje pomiedzy 225 a 270 st
          if(kierunek<=570)
            kier=225;
          else
            kier=270;
          kier_min=1023;
          kier_max=0;
          return;
          yield();
        }
      if(kierunek <18) kier=90;   //przelicza wartosc analogowa na kierunek
      else if (kierunek >= 22 && kierunek < 35) kier=135;
      else if (kierunek > 40 && kierunek < 58) kier=180;
      else if (kierunek > 150 && kierunek < 220) kier=225;
      else if (kierunek > 950) kier=270;
      else if (kierunek > 600 && kierunek < 700) kier=315;
      else if (kierunek > 300 && kierunek < 430) kier=0;
      else if (kierunek > 75 && kierunek < 125) kier=45;
      kx += V*sin(kier*PI/180);
      ky += V*cos(kier*PI/180);
      //Serial.println(kier);
      kier_min=1023;
      kier_max=0;
    }
  yield();
}


String getTime() {
  WiFiClient client;
  yield();
  time0=millis();
  while (!!!client.connect("google.com", 80)) { //sprawdza dostepnosc serwera
    if((millis()-time0)<5000)                   //czeka 5s jesli nie dostepny
    {
      delay(500);
      Serial.println("connection failed, retrying...");
      yield();
    }
    else
    {
      return "0";
      yield();
    }
  }

  client.print("HEAD / HTTP/1.1\r\n\r\n");
  time4=millis();
  while(!!!client.available()) {
     yield();
     if((millis()-time4)>5000)
     {
        return "0";
        yield();
     }
  }

  while(client.available()){
    if (client.read() == '\n') {    
      if (client.read() == 'D') {    
        if (client.read() == 'a') {    
          if (client.read() == 't') {    
            if (client.read() == 'e') {    
              if (client.read() == ':') {    
                client.read();
                String theDate = client.readStringUntil('\r');
                client.stop();
                return theDate;
                yield();
              }
            }
          }
        }
      }
    }
  }
  yield();
}

int ktory_miesiac(String miesiac_)
{
  int mie=0;
  if(miesiac_ == "Jan") mie=1;
  else if(miesiac_ == "Feb") mie=2;
  else if(miesiac_ == "Mar") mie=3;
  else if(miesiac_ == "Apr") mie=4;
  else if(miesiac_ == "May") mie=5;
  else if(miesiac_ == "Jun") mie=6;
  else if(miesiac_ == "Jul") mie=7;
  else if(miesiac_ == "Aug") mie=8;
  else if(miesiac_ == "Sep") mie=9;
  else if(miesiac_ == "Oct") mie=10;
  else if(miesiac_ == "Nov") mie=11;
  else if(miesiac_ == "Dec") mie=12;
  return mie;
  yield();
}

String odpowiedz() {

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  yield();
  const int httpPort = 80;
  if (!client.connect("widmo.tele.biz", httpPort)) {
    yield();
    Serial.println("connection 2 not OK");
    error=1;
    return "1";
    yield();
  }

 client.println("HEAD /Home/Index/ HTTP/1.2");
  client.println("Host: widmo.tele.biz");
  client.println();
  time4=millis();
  while(!!!client.available()) {
     yield();
     if((millis()-time4)>10000)
     {
      error=1;
      return "1";
      yield();
     }
   yield();
  }

  while(client.available()){
    if (client.read() == 'H') {    
      if (client.read() == 'T') {    
        if (client.read() == 'T') {    
          if (client.read() == 'P') {    
            if (client.read() == '/') {    
              if (client.read() == '1') {    
                if (client.read() == '.') {
                  if (client.read() == '1') {
                    //if (client.read() == ' ') {
                          client.read();
                          client.read();
                          client.read();
                          client.read();
                          client.read();
                          String theDate = client.readStringUntil('\r');
                          client.stop();
                          return theDate;
                          yield();
                    //}
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  yield();
}


void data(){
      String date = getTime();
      if(date=="0")
      {
        Serial.println("Nie moge pobrac godziny");
        ESP.restart();
        delay(1000);
      }
      String rok_ = date.substring(14, 16);
      String miesiac_ = date.substring(8, 11);
      String dzien_ = date.substring(5, 7);
      String godzina_ = date.substring(17, 19);
      String minuta_ = date.substring(20, 22);
      String sekunda_ = date.substring(23, 25);
      rok = 2000+rok_.toInt();
      miesiac = ktory_miesiac(miesiac_);
      dzien = dzien_.toInt();
      godzina = godzina_.toInt();
      minuta = minuta_.toInt();
      sekunda = sekunda_.toInt();
      
      Serial.print("Godzina ");
      Serial.print(godzina);
      Serial.print(":");
      if(minuta<10)
        Serial.print("0");
      Serial.println(minuta);
      Serial.print("Data ");
      Serial.print(rok);
      if(miesiac<10)
        Serial.print("-0");
      else
        Serial.print("-");
      Serial.print(miesiac);
      if(dzien<10)
        Serial.print("-0");
      else
        Serial.print("-");
      Serial.println(dzien);
      yield();
}
void ustaw_godzine()
{
  tmElements_t tm;
  time_t t;
       
  tm.Year = CalendarYrToTm(rok);      
  tm.Month = miesiac;
  tm.Day = dzien;
  tm.Hour = godzina;
  tm.Minute = minuta;
  tm.Second = sekunda;
  t = makeTime(tm);
  //RTC.set(t);        //use the time_t value to ensure correct weekday is set
  setTime(t); 
  yield();
}

void policz_kier()
{

  float dokladnosc=0;
  if(ane==1)
  {
    dokladnosc=1;                         //zadajemy dokladnosc w st. z jaka chcemy otrzymac wynik
    if(kx==0 && ky==0)
      kier_sr=kier;
    else
    {
      if(kx>=0 && ky>=0)                    //sprawdzamy w ktorej cwiartce jest obliczana wartosc i wyliczamy ja
        kier_sr=atan(kx/ky)*180/PI;
      else if(ky<0)
        kier_sr=180+atan(kx/ky)*180/PI;
      else
        kier_sr=360+atan(kx/ky)*180/PI;
                                            //zaokraglamy obliczona wartosc do zadanej dokladnosci
      if(((kier_sr/dokladnosc)-float(int(kier_sr/dokladnosc)))>=0.5) kier_sr=(int(kier_sr/dokladnosc)+1)*dokladnosc;   //przelicza średni kat na wartosci co 45st, lub co 22.5
      else                                                           kier_sr=int(kier_sr/dokladnosc)*dokladnosc;
    }
  } 
  else
  {
    dokladnosc=1;
    if(kx==0 && ky==0)
      kier_sr=kier;
    else
    {
      if(kx>=0 && ky>=0)
        kier_sr=atan(kx/ky)*180/PI;
      else if(ky<0)
        kier_sr=180+atan(kx/ky)*180/PI;
      else
        kier_sr=360+atan(kx/ky)*180/PI;
      if(kier_sr==360)
        kier_sr=0;
      if((int(kier_sr)%int(dokladnosc))>=(dokladnosc/2))     kier_sr=(int(kier_sr/dokladnosc)+1)*dokladnosc;   //przelicza średni kat na wartosci co 45st, lub co 22.5
      else                                                   kier_sr=int(kier_sr/dokladnosc)*dokladnosc;
    }
  }
    yield();
}
