from solver import solver

if __name__ == "__main__":


    n=-1
    while not(n>0):
        n = int(input("Ingrese el número de nodos: "))

    p=-1
    while not(p<=1 and p>=0):
        p = float(input("Ingrese la probabilidad de conexión entre nodos: "))

    for i in range(2):
        solver(n,p)

