#!/usr/bin/env python

#  Testing of Microsoft DialoGPT conversational model using hugging face transformers

import traceback

from transformers import AutoTokenizer, AutoModelForCausalLM
#from transformers import BlenderbotTokenizer, BlenderbotForConditionalGeneration
import torch
import time

PERIOD = 0.01 # update time in seconds
MODEL = "microsoft/DialoGPT-small"
#MODEL = "facebook/blenderbot-400M-distill"
class DialoGPT():
    def __init__(self):
        name = 'dialogpt test'
        self.tokenizer = AutoTokenizer.from_pretrained(MODEL)
        #self.tokenizer = BlenderbotTokenizer.from_pretrained(MODEL)
        device = "cuda:0" if torch.cuda.is_available() else "cpu"
        print('device" {}'.format(device))
        self.model = AutoModelForCausalLM.from_pretrained(MODEL)
        #self.model = BlenderbotForConditionalGeneration.from_pretrained(MODEL)
        print(name + ' done with init')

        # Testing
        # https://huggingface.co/microsoft/DialoGPT-medium
        # Let's chat for 5 lines
        seed = "Do robots dream about elephants?" 
        length = 0
        start = time.time()
        chat_ids = self.tokenizer.encode(seed + self.tokenizer.eos_token, return_tensors='pt')
        for step in range(5):
            # encode the new user input, add the eos_token and return a tensor in Pytorch

            # generated a response while limiting the total chat history to 1000 tokens, 
            # pretty print last ouput tokens from bot
            print("sec: {:.2f} response: {}".format(time.time() - start, self.tokenizer.decode(chat_ids[:, length:][0], skip_special_tokens=True)))
            #print("sec: {:.2f} response: {}".format(time.time() - start, self.tokenizer.decode(chat_ids[0], skip_special_tokens=True)))
            length = chat_ids.shape[1]
            start = time.time()
            chat_ids = self.model.generate(chat_ids, max_length=1000, pad_token_id=self.tokenizer.eos_token_id)

            """
            idses = []
            #texts = ['Do you like movies?', 'What is your favorite movie?', 'My favorite is Star Wars.']
            #texts = ['Are you a robot?', 'But are you a robot?', "I'm sure you are a robot."]
            texts = ["I'm feeling kind of bored"]
            print('Seed: {}'.format(texts))
            for text in texts:
                if device != 'cpu':
                    context = self.tokenizer.encode(text + self.tokenizer.eos_token, return_tensors='pt').cuda()
                else:
                    context = self.tokenizer.encode(text + self.tokenizer.eos_token, return_tensors='pt')
                    idses.append(context)
            count = 8
            while True:
                start = time.time()
                # encode the new user input, add the eos_token and return a tensor in Pytorch

                bot_input_ids = context
                # generate a response while limiting the total chat history to 1000 tokens, 
                new_ids = self.model.generate(
                    bot_input_ids, 
                    max_length=500, 
                    pad_token_id=self.tokenizer.eos_token_id,
                    repetition_penalty=1.4
                )
                response_ids = new_ids[:, bot_input_ids.shape[-1]:]
                #print('new_ids.shape: {} response_ids.shape {}'.format(new_ids.shape, response_ids.shape))
                response = self.tokenizer.decode(response_ids[0], skip_special_tokens=True)

                # keep count previous statements
                idses.append(response_ids)
                while len(idses) > count:
                    idses.pop(0)
                context = idses[0]
                for i in range(1, len(idses)):
                    context = torch.cat([context, idses[i]], dim=-1)
                #print('context.shape {}'.format(context.shape))
                print('({:6.4f}) DialoGPT: {}'.format(time.time()-start, response))
            """
        exit()

def main():
    dialogpt = DialoGPT()

if __name__ == '__main__':
    main()
