import requests
import json
import sys
sys.path.append('..')
import threadmanager
import config
import threading

class CommunicationWithServer:
    def __init__(self) :
        self.threadManager = threadmanager.CustomThread()
        # Ime lahko poenostavis preprosto v locker - statesObjectsLocker sem poimenoval Lock() objekt v app.py samo zato, ker lock-a vse spremenljivke stanja robota (kote, pozicije nog, tokove, ...).
        self.statesObjectsLocker = threading.Lock()

    # V celotnem programu je veliko 'pozicij', npr. pozicija pajka, pozicija pajkove noge, pozicija pina, pozicija roze itd. Izberi taksno ime, da se bo tocno vedelo, da gre za pozicijo roze.  
    def updatePositionData(self):
        """Comunication procedure, creates a thread that continuously sends GET requests to the server and updates the data variable with values from the server.
        """
        def updatingPositionData(killEvent):
            # Uporabi while True (samo zaradi konsistentnosti, nic drugega).
            while 1:
                try:
                    # Definiraj ta string naslova kot konstanto. Ce bo do te konstante moral dostopati se kak drug objekt, ki ne bo inicializiral nove instance CommunicationWithServer objekta, potem definiraj
                    # konstanto v config.py file-u. Ce ne, jo lahko definiras tukaj, na zacetku __init__ (definicija konstant pride pred ostalimi definicijami + poimenuj konstanto v stilu TO_JE_KONSTANTA).
                    request = requests.get('http://192.168.1.20:5000/zalij')
                    # Presledki pred in po operatorju.
                    if len(request._content.decode())!=0:
                        # 1.) Tukaj je potrebno uporabiti lock objekt (imas dve moznosti - self.statesObjectsLocker.aquire() + self.statesObjectsLocker.release() 
                        # ali pa (kar je bolj 'sodobno' in se naceloma uporablja) with self.statesObjectsLocker: ...). 
                        # Kot je program sedaj zastavljen, ta thread neprestano zapisuje v self.data, v app.py pa se na vsake toliko ta ista spremenljivka (oz. vsebina istega naslova v pomnilniku) prebere.
                        # Da ne pride do nepredvidljivih stvari v primeru, da se ob istem trenutku v enem threadu zapise, v drugem pa prebere vsebino istega naslova, se uporabi locker, ki 'zaklene' dostop do tega naslova.
                        # 2.) Sam sem vse deklaracije spremenljivk objekta (self.) dajal v __init__ metodo da se takoj ve, katere so spremenljivke objekta in so dostopne navzven. Ce ob deklaraciji nima vrednosti, 
                        # ji lahko das vrednost None.
                        # 3.) Ime self.data ne nakazuje, da sta v tej spremenljivki shranjena podatka o senzorju in vrstici (na koncu bo to podatek o dejanski x, y, z poziciji rastline, ki potrebuje zalivanje).
                        self.data=json.loads(request._content)
                # Tukaj lovis vse mogoce exceptione, do katerih lahko pride pri klicih get(), decode(), len() in loads() metod. 
                # Ne pravim, da to ni OK, samo mogoce si vzami par minut in premisli, ce bi kateri od teh zahteval kaksen poseben handling.
                except:
                    print("will retry in a second")
                # Timeout tukaj je lahko brez problema 30s ali se vec.
                if killEvent.wait(timeout = 2): 
                    break
        # Sam thread ne rabi biti spremenljivka objekta (self.). Navzven mora biti dostopen samo event, ki ubije thread (po c-jevsko: 'public' mora biti samo event, da lahko iz drugje ubijes ta thread).
        self.updatingDataThread, self.updatingDataThreadKillEvent = self.threadManager.run(updatingPositionData, config.UPDATE_DATA_THREAD_NAME, False, True)
