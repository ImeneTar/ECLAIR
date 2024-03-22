import numpy as np


def argmax(q_values):
    top = float("-inf")
    ties = []

    for i in range(len(q_values)):
        if q_values[i] > top:
            top = q_values[i]
            ties = []

        if q_values[i] == top:
            ties.append(i)

    return np.random.choice(ties)

class Q_learning:
    def __init__(self, alpha=0.1, gamma=0, eps=0, nEpisodes=30, timeout=20, width=5, action_space=6):
        self.alpha = alpha
        self.gamma = gamma
        self.eps = eps
        self.nEpisodes = nEpisodes
        self.timeout = timeout
        self.nb_features = width * width * 2
        self.nb_actions = action_space
        self.w = np.zeros((self.nb_actions, self.nb_features))


    def select_action(self, x, advice):

        if advice !='None' and advice != None and advice != " None":
            print("Advice is not none:", advice)
            return int(advice)
        else:
            # epsilon greedy search
            action_values = []
            for a in range(self.nb_actions):
                # only one feature activated other 0
                action_values.append(self.w[a][x])

            if np.random.random() < self.eps:
                chosen_action = np.random.choice(self.nb_actions)
            else:
                chosen_action = np.argmax(action_values)

            return chosen_action

    def update(self, x, u, evaluative, corrective):
        if evaluative != 'None' or int(evaluative) != 0 : #evaluative feedback
            action_value = self.w[u, x]
            self.w[u][x] += self.alpha * (int(evaluative) - action_value)
        if corrective != 'None' and corrective != ' None' : #corrective feedback (increase the probability of the action that should be taken instead)
            action_value = self.w[int(corrective), x]
            self.w[int(corrective)][x] += self.alpha * (1 - action_value)

