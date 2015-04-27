'*************************************************************************
'Der folgende Code dient zum Auszulesen eines Zeilensensors der Fa. Taos.
'Der Baustein TSL1401R-LF bietet eine Auflösung von 400 Dpi.
'
'Mit den zwei Taktsignalen, SI und Clk wird die Ausgabe der 128 Fotodioden
'gesteuert.
'
'Die Daten werden an die serielle Schnittstelle übergeben.
'
'Es lassen sich Kanten erfassen (Linien nachfahren; Strichcodes können erkannt
'werden;
'oder es lassen sich, bei entsprechender optischer Ausrüstung (Gitter/Prisma),
'auch spektroskopische Daten erfassen.
'
'Es gibt auch pinkompatibele Typen als SMD Bauteile, die sich auf sehr kleinem
'Raum unterbringen lassen.
'
'Der Code wurde noch nicht auf Geschwindigkeit optimiert aber über die NOP's
'läßt sich sicher noch etwas machen.
'*************************************************************************


$regfile = "m8def.dat"
$crystal = 3686400
$baud = 38400

Dim Buffer(128) As Byte
Dim Exposure As Word
Dim Integrate_time As Word
Dim N As Byte

'...

Clk Alias Portb.1
Si Alias Portb.0


Macro Clock_si
  Set Si : Nop : Nop : Nop : Nop
    Set Clk : Nop : Nop : Nop : Nop
      Reset Clk
        Reset Si
End Macro

Macro Send_clock
  Set Clk : Nop : Nop : Nop : Nop
    Reset Clk : Nop : Nop : Nop : Nop
End Macro


Exposure = 8333                                             ' Belichtungszeit in µS
Integrate_time = Exposure
  If Integrate_time < 2032 Then Integrate_time = 2032
    Integrate_time = Integrate_time - 2032

'**************************
'Initialisierung
'**************************

'Port C alles Eingänge
Ddrc = &B00000000
'Port C alle Eingänge bis auf Port Nr. Null auf 5V ziehen
Portc = &B11111110
'Port B Pin 0 / 1 Ausgänge
Ddrb = &B00000011

Config Adc = Single , Prescaler = 8 , Reference = Avcc
Set Admux.5
Start Adc
'**************************


Do

Clock_si
  For N = 0 To 150
    Send_clock
  Next N
    Set Si
      Waitus Integrate_time
    Reset Si
      Clock_si
        For N = 0 To 128
          Send_clock
            Set Adcsra.adsc
              While Adcsra.adsc = 1 : Wend
            Buffer(n) = Adch
        Next N



'************************************************
'Daten an serielle Schnittstelle
'Ausgabe der Pixelnummer und des Meßwertes
'************************************************
For N = 0 To 128
   If N < 128 Then
      Print N ; ";" ; Buffer(n)
      Waitms 10
   End If
Next N
'************************************************

Loop


