# TriacExpander

**RS485 Adresabilni 16-kanalni Triac Ekspander**

## Opis

TriacExpander je hardverski modul i firmware rešenje koje omogućava upravljanje do 16 AC kanala putem RS485 komunikacije. Svaki kanal je kontrolisan pomoću triaka, što omogućava efikasno i bezbedno uključivanje/isključivanje AC potrošača. Adresabilnost modula omogućava povezivanje više ekspandera na istu RS485 magistralu.

## Funkcionalnosti

- **16 nezavisnih AC kanala** – svaki kanal se može pojedinačno uključiti ili isključiti
- **RS485 komunikacija** – robustan i pouzdan industrijski standard za serijsku komunikaciju na duže udaljenosti
- **Adresabilni moduli** – svaki ekspander ima sopstvenu adresu na mreži
- **Podrška za kaskadno povezivanje** – više modula može raditi zajedno
- **Jednostavna integracija** – lako se povezuje sa PLC-om, mikrokontrolerima (npr. Arduino, ESP32), ili PC aplikacijama

## Tehničke karakteristike

- **Napajanje:** 220V AC (za triac kanale), 12V DC (za logiku, zavisno od dizajna)
- **Komunikacija:** RS485 (Modbus RTU ili custom protokol)
- **Broj kanala:** 16
- **Tip triaka:** BT136 ili ekvivalent
- **Maksimalna struja po kanalu:** do 16A (zavisno od triaka i hlađenja)
- **Mikrokontroler:** [STM32F030C6T6]

## Uputstvo za korišćenje

1. **Povezivanje modula**
    - Priključite AC izlaze na željene potrošače (npr. lampe, pumpe, grejače)
    - Povežite RS485 magistralu sa master uređajem (PLC, PC, Arduino...)

2. **Podešavanje adrese**
    - Svaki modul ima jumper/switch za podešavanje adrese (npr. 1-15)
    - Adresa se mora razlikovati za svaki modul na mreži

3. **Komunikacija**
    - Koristite odgovarajući komandni set za uključivanje/isključivanje kanala:
        - `SET <ADRESA> <KANAL> <ON/OFF>`
        - Primer: `SET 02 05 ON` – uključuje kanal 5 na modulu sa adresom 02

4. **Integracija sa softverom**
    - Primer Arduino koda za upravljanje:
    ```cpp
    // Slanje komande preko RS485
    Serial.print("SET 01 03 ON\n");
    ```

## Primeri primene

- Automatizacija rasvete u industriji ili domaćinstvu
- Upravljački sistemi grejanja, ventilacije i klimatizacije (HVAC)
- Kontrola pumpi, ventilatora i drugih električnih uređaja

## Šema povezivanja

*(https://github.com/eldar6776/TriacExpander/blob/main/hw/DE-140824/DE-140824.pdf)*

## Kontakt & Podrška

Za pitanja, sugestije ili podršku, kontaktirajte autora putem GitHub Issues

---

**Napomena:** Prilikom povezivanja AC tereta, obavezno poštujte sigurnosne mere i preporučuje se rad stručnog lica!
