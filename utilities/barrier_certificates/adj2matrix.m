function adj_matrix = adj2matrix(adj)

G_graph = graph(adj);
adj_matrix = table2array(G_graph.Edges);
adj_matrix = adj_matrix(:,1:2);  


end