import numpy as np
import matplotlib.pyplot as plt

population = 1000
nb_unities = 100
quantities_scale = 1000
ENERGY = 0
RESSOURCES = [int(quantity) for quantity in np.random.rand(nb_unities)*quantities_scale]
CELLS = []

GIFT = True
treshold = 100

class cell():
    def __init__(self,needs, prods):
        self.needs = needs # dict unity:quantity
        self.prods = prods # dict unity:quantity
        self.stock = {unity:0 for unity in self.needs.keys()}
        self.energy = 0

    def produce(self):
        global RESSOURCES
        global ENERGY
        full = True
        new_stock = {}
        for unity,quantity in self.stock.items():
            if quantity>=self.needs[unity]:
                new_stock[unity] = quantity - self.needs[unity]
            else:
                full = False
                break
        if full:
            self.stock = new_stock
            for unity,quantity in self.prods.items():
                RESSOURCES[unity]+=quantity

                if GIFT:
                    if self.energy<treshold:
                        self.energy += quantity
                        ENERGY -= quantity
                    else:
                        ENERGY += quantity
                else:
                    self.energy += quantity

    def consum(self):
        global RESSOURCES
        for unity,quantity in self.needs.items():
            if quantity<=RESSOURCES[unity]:
                RESSOURCES[unity] -= quantity
                self.stock[unity] += quantity

                if not GIFT:
                    self.energy -= quantity

            elif RESSOURCES[unity]>0:
                self.stock[unity] += RESSOURCES[unity]
                RESSOURCES[unity] = 0

                if not GIFT:
                    self.energy -= quantity

def make_cells():
    global CELLS
    for _ in range(population):
        len_needs = int(np.random.rand()*nb_unities)
        len_prods = int(np.random.rand()*nb_unities)
        unity_needs = np.random.choice(nb_unities,len_needs)
        unity_prods = np.random.choice(nb_unities,len_prods)
        needs = {unity:int(1+np.random.rand()*quantities_scale/float(population)) for unity in unity_needs}
        prods = {unity:int(1+np.random.rand()*quantities_scale/float(population)) for unity in unity_prods}
        #print("new cell : prods="+str(prods))
        #print("new cell : needs="+str(needs))
        CELLS.append(cell(needs,prods))

make_cells()

horizon = 1000
production = []
for t in range(horizon):
    total_energy = 0
    for cell in CELLS:
        cell.consum()
    for cell in CELLS:
        cell.produce()
        total_energy += cell.energy
    #total_energy += ENERGY
    production.append(total_energy)

cell_scores = [cell.energy+ENERGY/float(population) for cell in CELLS]
plt.plot(production)
plt.figure()
plt.plot(cell_scores)
plt.figure()
plt.plot(RESSOURCES)
plt.show()
