import re
import ollama
import pandas as pd
import chromadb
from chromadb.config import Settings
from chromadb.utils import embedding_functions
import rclpy
from rclpy.node import Node
from ltm_package.srv import LtmQuery, LtmSetModel, LtmRemember

class LongTermMemoryService(Node):
    def __init__(self):   
        super().__init__("MemoryNode") 
        self.gen_model = "granite3-moe:3b"
        self.embed_model = "granite-embedding:30m"
        self.collection_name = "k9_ltm_gran_30m"
        self.chroma_client = chromadb.PersistentClient(path="./chroma_db")
        self.collection = self.chroma_client.get_or_create_collection(
            name = self.collection_name
        )
        self._node = rclpy.create_node('long_term_memory_service')
        self._set_model_srv = self._node.create_service(LtmSetModel, 'ltm_set_model', self.set_model_callback)
        self._query_srv = self._node.create_service(LtmQuery, 'ltm_query', self.query_callback)
        self._remember_srv = self._node.create_service(LtmRemember, 'ltm_remember', self.remember_callback)
        df = pd.read_csv('k9_stories_500.csv')
        print(f"{len(df)} rows in the data.")
        df.sample(5)
        self.add_documents(df['synopsis'])

    def set_model_callback(self, request: LtmSetModel.Request, response: LtmSetModel.Response):
        response.success = False
        if request.model_type == 0: # GEN_MODEL
            self.gen_model = request.model_name
            response.success = True
        if request.model_type == 1: # GEN_MODEL
            self.embed_model_model = request.model_name
            response.success = True
        return response
    
    def retrieve_document(self, query):
        """
        Retrieve the most relevant document
        """
        response = ollama.embed(
            model=self.embed_model,
            input = query
        )
        results = self.collection.query(
            query_embeddings=response["embeddings"],
            n_results=1
        )
        if results["documents"]:
            return results["documents"][0][0]  # Return top match
        return None

    def extract_string(self, input_str):
        match = re.search('"([^"]*)"', input_str)
        if match is not None:
            return match.group(1)
        else:
            return input_str

    def query_callback(self, request: LtmQuery.Request, response: LtmQuery.Response):
        data = self.retrieve_document(query = request)
        if data is None:
            response.success = False
            response.answer = "Insufficient data"
            return response
        # generate a response combining the prompt and data we retrieved above
        output = ollama.generate(
            model=self.gen_model,
            prompt=f"You are K9, a robot dog. Using only this data: {data} generate K9's short, single sentence, response to this question: {request}. If there is not enough information to answer the question, respond with 'Insufficient data'."
            )
        if "insuffient data" in output.lower():
            response.success = False
        else:
            response.success = True
        response.answer = self.extract_string(output)
        return response
        
    def add_documents(self, documents):
        """
        Add documents to ChromaDB
        """
        for i, doc in enumerate(documents):
            response = ollama.embed(model=self.embed_model, input=doc)
            embeddings = response["embeddings"]
            self.collection.add(
                ids=[str(i)],
                embeddings=embeddings,
                documents=[doc]
            )
        print(f"Added {len(documents)} documents to K9's long term memory.")

    def remember_callback(self, request: LtmRemember.Request, response: LtmRemember.Response):
        facts = request.up_to_twenty_topics_of_512_chars
        num_facts = len(facts)
        response.success = False
        response.facts_remembered = 0
        if  num_facts > 20 or num_facts < 1 :
            return response
        for fact in facts:
            if len(fact) > 512:
                return response
        self.add_documents(facts)
        response.success = True
        response.facts_remembered = num_facts # changed the message to be more descriptive
        return response
    
    def main():
        rclpy.init()
        ltm_service = LongTermMemoryService()
        rclpy.spin(ltm_service)
        rclpy.shutdown()

if __name__ == '__main__':
    main()