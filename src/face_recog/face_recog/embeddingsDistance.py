import json
import numpy as np

# Load the embeddings from the JSON file
with open("embeddingsTiny.json", "r") as f:
    embeddings = json.load(f)

# Extract embeddings for Armaan and Wiktor
armaan_embedding = np.array(embeddings["Armaan"])
wiktor_embedding = np.array(embeddings["Wiktor"])

# Calculate the Euclidean distance between the embeddings
distance = np.linalg.norm(armaan_embedding - wiktor_embedding)

# Print the result
print(f"Euclidean distance between Armaan and Wiktor embeddings: {distance:.3f}")