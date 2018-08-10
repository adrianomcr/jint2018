import matplotlib.pyplot as plt
import numpy as np
import matplotlib.gridspec as gridspec
import time
from matplotlib import cm
from IPython import display
#from sklearn.datasets import make_classifiction
#from sklearn.cluster import KMeans
#from sklearn.datasets import load_sample_image
#from sklearn.cluster import MiniBatchKMeans
from random import randrange
import pylab

# Calculo de centro
def calcCenter(X,k,group_index,centers):
  #n,d = X.shape
  #centers = np.zeros((k,d))
  
  for i in range(0,k):
    vec_idx = (group_index==i)
    if( np.sum(vec_idx) > 0 ):
      centers[i] = np.mean(X[vec_idx], axis=0)
    else:
      print('Centro isolado.')



# Calculo da funcao objetivo
def calcObj(X,k,group_index):
  J = 0.0

  power = [1.0,1.0,1.5]

  for i in range(0,k):
    vaux = X[group_index==i]

    for j in range(0,vaux.shape[0]):
      J = J + np.sum( np.linalg.norm(vaux-vaux[j]) )/power[i]

  return J





def kmeans(X, k, epsilon = 1e-8, max_iter = 50):
    n,d = X.shape
    
    # PASSO 1 - Escolha k centros a partir de padroes aleatorios e armazene na matriz
    # center, de dimensoes kxd
    
    # Vetor com o grupo de cada elemento
    group_idx = np.zeros(n)    
    
    # Sorteia grupos para os elementos
    for i in range(0,n):
        group_idx[i] = np.random.randint(0,k)
    
    center = np.zeros((k,d))
    calcCenter(X,k,group_idx,center)
    #print(center)
    
    # Matriz de distancias de dimensao nxk
    dist = np.zeros((n,k))
    
    plt.figure(figsize=(15,5))
    if (d==2):
      gs = gridspec.GridSpec(1,2)
    oldJ = 0
    
    iteration = 0
    j_vec = np.zeros(max_iter+1)
    
    
    ## PLOTA OS RESULTADOS PARCIAIS        
    if (d==2):
      plt.subplot(gs[0])
      plt.cla()
      plt.scatter(X[:,0],X[:,1], c=group_idx+1)
      plt.plot(center[:,0],center[:,1], 'r<', markersize=10)
      plt.subplot(gs[1])


    display.clear_output(wait=True)
    display.display(plt.gcf())      
    
    while (iteration < max_iter):
      
        print( 'iteration = ' + str(iteration) )
        
        # PASSO 2 - Associe cada amostra ao grupo mais proximo
        # - Calcula a distancia entre as amostras e os centros dos grupos e 
        # armazena na matriz dist.
        #<SEU CODIGO AQUI>

        power = [1.0, 1.0, 1.8]
        print( 'Inicio PASSO 2 calcDist e gorupIndex ...')
        for i in range(0,n):
          for j in range(0,k):
            dist[i,j] = np.linalg.norm(X[i]-center[j])/power[j]
            
        #print('dist = ')
        #print(dist[:20])
        
        
        # - Encontra o centro de grupo mais proximo de cada amostra e armazena 
        # no vetor group_idx de tamanho n.
        for i in range(0,n):
          group_idx[i]= np.argmin(dist[i])
        #print(group_idx)
        print( '... Fim PASSO 2.')
        
        # PASSO 3 - Recalcule os centros dos grupos a partir do novo 
        # partcionamento das amostras e armazene o resultado na  matriz center
        # <SEU CODIGO AQUI>    
        print( 'Inicio PASSO 3 - calcCenter ...')
        calcCenter(X,k,group_idx,center)
        #print('center = ')
        #print(center)
        print( '... Fim PASSO 3.')

        # PASSO 4 - Pare se o criterio de convergencia foi atingido, caso  
        # contrario va para o Passo 2
        
        # - Calcula a funcao objetivo e armazene no escalar J
        # <SEU CODIGO AQUI>
        print( 'Inicio PASSO 4 - calcObj ...')
        J = calcObj(X,k,group_idx)
        print( '... Fim PASSO 4.')

        #print( 'iteration = ' + str(iteration) )
        #print( 'J = ' + str(J) )
        #time.sleep(2)
        
        j_vec[iteration] = J
        
        ## PLOTA OS RESULTADOS PARCIAIS        
        if (d==2):
          
          plt.subplot(gs[0])
          plt.cla()
          plt.scatter(X[:,0],X[:,1], c=group_idx+1)
          plt.plot(center[:,0],center[:,1], 'r<', markersize=10)
          plt.title('Iteration: %d'%(iteration))
        
          plt.subplot(gs[1])
          
        plt.cla()
        plt.plot(j_vec[:iteration])
        plt.title('Iteration: %d - J = %.4f'%(iteration,J))
        
        display.clear_output(wait=True)
        display.display(plt.gcf())           
            
          
        # - Checa se a variacao na funcao objetivo foi significativa
        if (np.linalg.norm(J-oldJ) <= epsilon):
            break;
        
        oldJ = J
        iteration = iteration + 1
        
    display.clear_output(wait=True)
    return center, group_idx, J





#X,_ = make_classification(n_samples= 100, n_features=2, n_classes = 4, n_clusters_per_class=1,n_informative=2, n_redundant=0, n_repeated=0, class_sep = 2)
pts = []
n = 400
for k in range(n):
    #pts.append([(6/1000.0)*randrange(0,1000,1)-3, (4/1000.0)*randrange(0,1000,1)-2])
    x = (6/1000.0)*randrange(0,1000,1)-3
    y = (4/1000.0)*randrange(0,1000,1)-2
    if (x**2+y**2<4):
        pts.append([x,y])
X = np.array(pts)
centers, gi, J = kmeans(X,k=3)

pylab.show()

print '\n'
print 'Here is centers:\n', centers
print 'Here is gi:\n', gi
print 'Here is J:\n', J


print 'Here is gi[3]:\n', gi[3]



#"""
pylab.figure(100)
pylab.axis('equal')
colors = ['r','g','b','y','c','m','k','r','g','b','y','c','m','k','r','g','b','y','c','m','k','r','g','b','y','c','m','k']


for k in range(len(pts)):
    cor = colors[int(gi[k])]
    pylab.plot(pts[k][0], pts[k][1], 'o', markersize=10.0, color=cor)

pylab.show()
#"""






