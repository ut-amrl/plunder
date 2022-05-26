import json
import random

def make_random_example():
    distance_to_human = random.random() * 3  # [0, 3)
    start_state = random.choice(('STOP', 'GO'))
    if distance_to_human < 1:
        output_state = 'STOP'
    else:
        output_state = 'GO'
    return {
        'distance_to_human': {
            'dim': [1, 0, 0],
            'type': 'NUM',
            'name': 'distance_to_human',
            'value': distance_to_human
        },
        'start': {
            'dim': [0, 0, 0],
            'type': 'STATE',
            'name': 'start',
            'value': start_state
        },
        'output': {
            'dim': [0, 0, 0],
            'type': 'STATE',
            'name': 'output',
            'value': output_state
        }
    }

def main():
    examples = [make_random_example() for _ in range(100)]
    json_out = json.dumps(examples)
    with open('toy.json', 'w') as f:
        f.write(json_out)

if __name__ == '__main__':
    main()