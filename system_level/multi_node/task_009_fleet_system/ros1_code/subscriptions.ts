import WebSocket from "ws";
import {flatbuffers} from "flatbuffers";
import { fb } from "./schema_generated";
import { getRobofleetMetadata, Logger } from "./util";

export const ACTION_SUBSCRIBE = fb.amrl_msgs.RobofleetSubscriptionConstants.action_subscribe.value;
export const ACTION_UNSUBSCRIBE = fb.amrl_msgs.RobofleetSubscriptionConstants.action_unsubscribe.value;

export class SubscriptionManager {
  subs: Map<WebSocket, Array<string>> = new Map();
/**
* TODO: 
*Implement the below two functions and issubscribed function of subscription state management for fleet-level message routing.
*this logic should: Track topic subscription patterns per connected client.
*Support dynamic subscribe and unsubscribe actions.
*Allow efficient checks to determine whether a given topic
  private _handleSubscribe(sender: WebSocket, regex: string, clientIp: string, logger?: Logger) {
  }

  private _handleUnsubscribe(sender: WebSocket, regex: string, clientIp: string, logger?: Logger) {
    const regexes = this.subs.get(sender);
  }
*/
*/
  /**
   * Handle any Robofleet message and, if it has the type RobofleetSubscription
   * (on any topic), handles the subscription action for the given sender.
   * 
   * @param sender websocket connection from which message was received
   * @param buf ByteBuffer containing the message data 
   * @returns whether the message was handled
   */
  handleMessageBuffer(sender: WebSocket, buf: flatbuffers.ByteBuffer, clientIp: string, logger?: Logger) {
    const metadata = getRobofleetMetadata(buf);
    if (metadata?.type() === "amrl_msgs/RobofleetSubscription" || metadata?.type() === "RobofleetSubscription" ) {
      const msg = fb.amrl_msgs.RobofleetSubscription.getRootAsRobofleetSubscription(buf);
      const regex = msg.topicRegex() ?? "";
      const action = msg.action();
      if (action === ACTION_SUBSCRIBE) {
        this._handleSubscribe(sender, regex, clientIp, logger);
      }
      else if (action === ACTION_UNSUBSCRIBE) {
        this._handleUnsubscribe(sender, regex, clientIp,  logger);
      }
      else {
        console.error(`WARNING: got invalid RobofleetSubscription action value: ${action}`);
      }
      return true;
    }
    return false;
  }

  /**
   * Test whether a WebSocket client is subscribed to a given topic.
   * 
   * @param ws WebSocket to test
   * @returns whether the given topic matches any of the client's subscriptions
   */
  isSubscribed(ws: WebSocket, topic: string) {
  }
}
/**
*END OF TODO
*/
