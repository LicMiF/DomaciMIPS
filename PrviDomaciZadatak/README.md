# PWM domaći zadatak iz mikroprocesorskih sistema. 

Zadatak je fokusiran na implementaciji PWM (Pulse Width Modulation). Potrebno je, u okviru Proteus-a, dodati LED diodu koja će simulirati rad impulsno širinske modulacije, dodati otpornik (koji bi trebalo da poseduje odgovarajuću snagu disipacije kao i otpornost), dodati uzemljenje i povezati anodu diode na odgovarajući pin.

Port mikrokontrolera (STM32F103C6) se bira u skladu sa brojem samoglasnika/suglasnika u imenu i prezimenu autora (Filip Milić). Kako je u postavci navedeno, ukoliko je broj suglasnika veći od broja samoglasnika, za port se uzima port B, a kako je upravo to slučaj sa autorovim imenenom i prezimenom, isti se odlučio za port B.

Pin porta se bira kao moduo zbira slova u imenu i prezimenu i broja 6 (BROJ_SLOVA_IME+BROJ_SLOVA_PREZIME)%6=(5+5)=4. Sada kada znamo port i pin na koji je potrebno povezati anodu, to činimo u okviru softverskog alata Proteus, kao i u konfiguraciji pinova pri kreiranju projekta u STMcubeIDE.

Preostalo je samo opredeliti se za intervale pauze kao i aktivnog naponskog nivoa impulsno širinske modulacije. U postavci je navedeno da se dati intervali biraju u skladu sa brojem slova u imenu i prezimenu, a kako je u autorovom slučaju taj broj jednak onda će i vreme trajanja pauze i aktivnog naposkog nivoa biti isto (5ms).
Širina impulsa će biti 50% ciklusa.

Kako je dalje u postavci navedeno, potrebno je nakon određenog broja ciklusa promeniti širinu impulsa tako što će sada vreme trajanja pauze biti vreme trajanja aktivnog naponskog nivoa a vreme trajanja aktivnog naponskog nivoa će sada postati vreme trajanja pauze. Broj ciklusa nakon koga se dešava ova promena jednak je broju slova u imenu (5) i kako bi došlo do naredne zamene dužina intervala impulsa i pauze potrebno je da prođe broj ciklusa jednak broju slova u prezimenu(5). Ovaj proces se ponavlja naizmenično.

Za realizaciju traženog efekta, koristila se funkcija HAL_Delay() biblioteke HAL. 
