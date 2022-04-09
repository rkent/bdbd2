#!/usr/bin/env python

#  Testing of Microsoft DialoGPT conversational model using hugging face transformers

import traceback

from transformers import AutoTokenizer, AutoModelForCausalLM
import statistics
#from transformers import BlenderbotTokenizer, BlenderbotForConditionalGeneration
import torch
import time

PERIOD = 0.01 # update time in seconds
MODEL = "microsoft/DialoGPT-small"
MAX_LENGTH = 50
CONVERSATION_LENGTH = 10
MAX_TIME = 10.0
REP_PEN = 1.002
USE_CUDA = True
#MODEL = "microsoft/DialoGPT-medium"
#MODEL = "facebook/blenderbot-400M-distill"
class DialoGPT():
    def __init__(self):
        name = 'dialogpt test'
        self.tokenizer = AutoTokenizer.from_pretrained(MODEL)
        #self.tokenizer = BlenderbotTokenizer.from_pretrained(MODEL)
        device = "cuda:0" if torch.cuda.is_available() and USE_CUDA else "cpu"
        print('device: {}'.format(device))
        self.model = AutoModelForCausalLM.from_pretrained(MODEL)
        if device != 'cpu':
            self.model = self.model.to(device)
        print(f'model type: {type(self.model)}')
        #self.model = BlenderbotForConditionalGeneration.from_pretrained(MODEL)
        print(name + ' done with init')

        # Testing
        # https://huggingface.co/microsoft/DialoGPT-medium
        # Let's chat for 5 lines
        seed = "Do robots dream about elephants?" 
        length = 0
        start = time.time()
        chat_ids = self.tokenizer.encode(seed + self.tokenizer.eos_token, return_tensors='pt')
        print(f'chat_ids: {chat_ids}')
        times = []
        chat_ids = chat_ids.to(device)
        for step in range(CONVERSATION_LENGTH):
            # encode the new user input, add the eos_token and return a tensor in Pytorch

            # generated a response while limiting the total chat history to 1000 tokens, 
            # pretty print last ouput tokens from bot
            print("sec: {:.2f} response: {}".format(time.time() - start, self.tokenizer.decode(chat_ids[:, length:][0], skip_special_tokens=True)))
            times.append(time.time() - start)
            #print("sec: {:.2f} response: {}".format(time.time() - start, self.tokenizer.decode(chat_ids[0], skip_special_tokens=True)))
            length = chat_ids.shape[1]
            start = time.time()
            chat_length = chat_ids.shape[1]
            chat_ids = self.model.generate(
                chat_ids,
                max_length=chat_length + MAX_LENGTH,
                pad_token_id=self.tokenizer.eos_token_id,
                repetition_penalty=REP_PEN,
                max_time = 10.0
            )
            print(f'chat_ids.shape: {chat_ids.shape}')
        print(f'Last 3 times mean: {statistics.mean(times[-3:])}')
        exit()

def main():
    dialogpt = DialoGPT()

if __name__ == '__main__':
    main()
