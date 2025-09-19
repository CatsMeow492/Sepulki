import { 
  ApolloClient, 
  InMemoryCache, 
  createHttpLink,
  from,
  ApolloLink
} from '@apollo/client';
import { setContext } from '@apollo/client/link/context';
import { onError } from '@apollo/client/link/error';
import { WebSocketLink } from '@apollo/client/link/ws';
import { getMainDefinition } from '@apollo/client/utilities';
import { createClient as createWSClient } from 'graphql-ws';

import { AuthManager } from './auth';
import type { SepulkiClientConfig } from './types';

export class SepulkiClient {
  public apollo: ApolloClient<any>;
  public auth: AuthManager;
  
  constructor(config: SepulkiClientConfig) {
    this.auth = new AuthManager({
      apiUrl: config.apiUrl,
      storage: config.storage || (typeof window !== 'undefined' ? localStorage : undefined)
    });

    // HTTP Link
    const httpLink = createHttpLink({
      uri: `${config.apiUrl}/graphql`
    });

    // WebSocket Link for subscriptions
    const wsLink = new WebSocketLink(
      createWSClient({
        url: config.websocketUrl || `${config.apiUrl.replace('http', 'ws')}/graphql`,
        connectionParams: () => ({
          authToken: this.auth.getToken()
        })
      })
    );

    // Auth Link
    const authLink = setContext(async (_, { headers }) => {
      const token = await this.auth.getToken();
      
      return {
        headers: {
          ...headers,
          authorization: token ? `Bearer ${token}` : ""
        }
      };
    });

    // Error Link
    const errorLink = onError(({ graphQLErrors, networkError, operation, forward }) => {
      if (graphQLErrors) {
        graphQLErrors.forEach(({ message, locations, path, extensions }) => {
          console.error(
            `[GraphQL error]: Message: ${message}, Location: ${locations}, Path: ${path}`
          );

          // Handle authentication errors
          if (extensions?.code === 'UNAUTHENTICATED') {
            this.auth.logout();
            
            if (config.onAuthError) {
              config.onAuthError(message);
            }
          }
        });
      }

      if (networkError) {
        console.error(`[Network error]: ${networkError}`);
        
        // Handle network authentication errors
        if ('statusCode' in networkError && networkError.statusCode === 401) {
          this.auth.logout();
          
          if (config.onAuthError) {
            config.onAuthError('Network authentication error');
          }
        }
      }
    });

    // Split Link (HTTP for queries/mutations, WebSocket for subscriptions)
    const splitLink = ApolloLink.split(
      ({ query }) => {
        const definition = getMainDefinition(query);
        return (
          definition.kind === 'OperationDefinition' &&
          definition.operation === 'subscription'
        );
      },
      wsLink,
      httpLink
    );

    // Create Apollo Client
    this.apollo = new ApolloClient({
      link: from([
        errorLink,
        authLink,
        splitLink
      ]),
      cache: new InMemoryCache({
        typePolicies: {
          Query: {
            fields: {
              sepulkas: {
                merge: false // Don't merge, replace
              },
              fleets: {
                merge: false
              },
              tasks: {
                merge: false
              }
            }
          }
        }
      }),
      defaultOptions: {
        watchQuery: {
          errorPolicy: 'all'
        }
      }
    });
  }

  // Convenience methods
  async login(email: string, password: string) {
    return this.auth.login(email, password);
  }

  async logout() {
    await this.apollo.clearStore();
    return this.auth.logout();
  }

  async getCurrentSmith() {
    return this.auth.getCurrentSmith();
  }

  isAuthenticated(): boolean {
    return this.auth.isAuthenticated();
  }

  // GraphQL operation helpers
  query = this.apollo.query.bind(this.apollo);
  mutate = this.apollo.mutate.bind(this.apollo);
  subscribe = this.apollo.subscribe.bind(this.apollo);
  watchQuery = this.apollo.watchQuery.bind(this.apollo);
}
