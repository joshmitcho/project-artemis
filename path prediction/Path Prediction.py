import time
import pylab
import pykalman

data = {
    "pos": [(0, 0, 0)] * 4,
    "vel": [(0, 0, 0)] * 4,
    "acc": [(0, 0, 0)] * 4,
    "jrk": [(0, 0, 0)] * 4,
}

help(pykalman)

newVel = [0]*3
newAcc = [0]*3
newJrk = [0]*3

nameList = ['pos', 'vel', 'acc', 'jrk']


def update(newPos):
    data["pos"] = [tuple(newPos)] + data["pos"][:-1]

    for i in range(len(data["vel"][0])):
        newVel[i] = data["pos"][0][i] - data["pos"][1][i]
    data["vel"] = [tuple(newVel)] + data["vel"][:-1]

    for i in range(len(data["acc"][0])):
        newAcc[i] = data["vel"][0][i] - data["vel"][1][i]
    data["acc"] = [tuple(newAcc)] + data["acc"][:-1]

    for i in range(len(data["jrk"][0])):
        newJrk[i] = data["acc"][0][i] - data["acc"][1][i]
    data["jrk"] = [tuple(newJrk)] + data["jrk"][:-1]


def simulate(i):
    out = []
    temp = i[1:-2].split(", ")
    for j in temp:
        out.append(float(j))
    return out


def buildSample():
    p = [0, 0.1, 0]
    v = [50, 100, 30]
    a = [-1, -9.8, -1]
    j = [0.01, 0.1, 0.01]

    infile = open("sample", 'w')
    for t in range(30):
        infile.write(str(p) + '\n')
        if p[1] > 0:
            p[0] += v[0]
            p[1] += v[1]
            p[2] += v[2]
            v[0] += a[0]
            v[1] += a[1]
            v[2] += a[2]
            a[0] += j[0]
            a[1] += j[1]
            a[2] += j[2]
    infile.close()


def predict(i, target):

    p, v, a, j = list(data['pos'][0]), list(data['vel'][0]), list(data['acc'][0]), list(data['jrk'][0])
    for i in range(target - i):
        p[0] += v[0]
        p[1] += v[1]
        p[2] += v[2]
        v[0] += a[0]
        v[1] += a[1]
        v[2] += a[2]
        a[0] += j[0]
        a[1] += j[1]
        a[2] += j[2]

    return p


def graph(x1, x2, name):
    pylab.plot(range(1, len(x1)+1), x1, 'ro')
    pylab.plot(range(1, len(x2)+1), x2, 'bs')
    pylab.title(name)
    pylab.legend(["Actual " + name, "Predicted " + name])
    pylab.show()


def graphD(x1, x2, x3, name):
    pylab.plot(range(1, len(x1)+1), x1, 'o')
    pylab.plot(range(1, len(x2)+1), x2, 'o')
    pylab.plot(range(1, len(x3) + 1), x3, 'o')
    pylab.title(name)
    pylab.legend(["Delta X", "Delta Y", "Delta Z"])
    pylab.show()

def main():
    start = time.time()
    target = 20
    buildSample()

    predX = []
    predY = []
    predZ = []

    x = []
    y = []
    z = []

    dx = []
    dy = []
    dz = []
    infile = open("sample", 'r')
    for i in range(30):

        update(simulate(infile.readline()))

        if i >= 3:
            dx.append(predict(i, target)[0])
            dy.append(predict(i, target)[1])
            dz.append(predict(i, target)[2])

            x.append(data['pos'][0][0])
            y.append(data['pos'][0][1])
            z.append(data['pos'][0][2])
            predX.append(predict(i, target)[0])
            predY.append(predict(i, target)[1])
            predZ.append(predict(i, target)[2])

        if i == target:
            for j in range(len(dy)):
                dx[j] -= data['pos'][0][0]
            for j in range(len(dy)):
                dy[j] -= data['pos'][0][1]
            for j in range(len(dz)):
                dz[j] -= data['pos'][0][2]
            break

        print('position at time ' + str(i) + ' = ' + str(data["pos"][0]))
        print('prediction for time ' + str(target) + ' = ' + str(predict(i, target)))
        print
    infile.close()
    duration = time.time() - start
    print('duration:', duration)

    graph(x, predX, 'X')
    graph(y, predY, 'Y')
    graph(z, predZ, 'Z')

    graphD(dx, dy, dz, 'Prediction Deltas')


#main()

